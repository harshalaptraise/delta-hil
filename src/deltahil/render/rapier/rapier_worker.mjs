/* Rapier rigid-body worker for RapierCellPlant (the Python side holds ALL cell logic;
 * this is a thin physics service). One JSON request per line on stdin -> one JSON line
 * on stdout. The Python plant owns spawn/TCP/grasp-gate/conservation; here we only run
 * the Rapier world: kinematic grippers + totes, dynamic tortillas, an explicit belt
 * Coulomb drag (Rapier has no native conveyor), weld = fixed joint on the gripper anchor.
 *
 * World is z-up (gravity -z), metres, matching the plant/cell_scene frame. Rapier
 * cylinders are y-axis aligned, so the tortilla collider is rotated so its axis is z
 * (disc lies flat at identity) -- the streamed body quaternion is then the honest tumble.
 *
 * Protocol (stdin, one JSON object per line):
 *   {geom:{...}}                       -> build the world, reply {ready:true}
 *   {dt, belt_v, grippers:[[x,y,z]..], totes:[[slot,x,y,z]..], spawn:[[slot,x,y,z]..],
 *    park:[slot..], weld:[[g,slot]..], unweld:[[g,slot]..], drag:[slot..]}
 *                                      -> step, reply {items:[[slot,x,y,z,qw,qx,qy,qz]..]}
 */
import RAPIER from './vendor/rapier.mjs';   // vendored @dimforge/rapier3d-compat (offline, no npm)
import readline from 'node:readline';

const G = 9.81, PARK = [7.0, 5.0];
let world, belt, items = [], totes = [], grips = [], welds = new Map();  // "g:slot" -> joint
let TR, THH, N_ITEMS, N_TOTES, ROT_X;

function qmul(a, b) {  // quaternion (w,x,y,z)
  return { w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
           x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
           y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
           z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}

function build(geom) {
  TR = geom.tr; THH = geom.thh; N_ITEMS = geom.n_items; N_TOTES = geom.n_totes;
  const tw = geom.tote_w, th = geom.tote_h, twall = geom.tote_wall;
  world = new RAPIER.World({ x: 0, y: 0, z: -G });
  world.integrationParameters.dt = 0.005;

  // floor
  world.createCollider(RAPIER.ColliderDesc.cuboid(5, 4, 0.05).setTranslation(0, 0, -0.05));
  // source belt: static slab (supports normal load; carry is explicit Coulomb drag)
  belt = world.createCollider(       // frictionless slab: it only carries the normal load; the
    RAPIER.ColliderDesc.cuboid(geom.belt_len / 2 + 0.25, 0.14, 0.02)   // scripted carry does the
      .setTranslation(0, geom.src_y, geom.src_top - 0.02)              // rest. Min-combine so the
      .setFriction(0.0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min));  // item's
  //   own friction (1.0, needed to pile in totes) never brakes it against the static belt

  // tortilla pool: dynamic cylinders, collider rotated so the disc axis is z (flat)
  ROT_X = { w: Math.cos(Math.PI / 4), x: Math.sin(Math.PI / 4), y: 0, z: 0 };  // +90 about x
  for (let i = 0; i < N_ITEMS; i++) {
    const b = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic().setTranslation(PARK[0] + 0.3 * i, PARK[1], 0.05)
        .setLinearDamping(0.0).setAngularDamping(0.4).setCanSleep(false));
    world.createCollider(
      RAPIER.ColliderDesc.cylinder(THH, TR).setRotation(ROT_X).setMass(0.05).setFriction(1.0), b);
    items.push(b);
  }
  // tote pool: kinematic open boxes (floor + 4 walls)
  for (let k = 0; k < N_TOTES; k++) {
    const b = world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(PARK[0] + 0.4 * k, PARK[1] + 1, 0.34));
    world.createCollider(RAPIER.ColliderDesc.cuboid(tw, tw, twall).setTranslation(0, 0, 0), b);
    for (const [sx, sy, w, d] of [[1, 0, twall, tw], [-1, 0, twall, tw], [0, 1, tw, twall], [0, -1, tw, twall]])
      world.createCollider(RAPIER.ColliderDesc.cuboid(w, d, th).setTranslation(sx * tw, sy * tw, th), b);
    totes.push(b);
  }
  // gripper anchors: kinematic, no collider (joint anchors only)
  for (let r = 0; r < geom.n_robots; r++)
    grips.push(world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(geom.robot_x[r], 0, 0.55)));
}

function setKin(body, x, y, z) { body.setNextKinematicTranslation({ x, y, z }); }

function doWeld(g, slot) {
  const key = `${g}:${slot}`;
  if (welds.has(key)) return;
  const it = items[slot], gr = grips[g], gp = gr.translation();
  it.setTranslation({ x: gp.x, y: gp.y, z: gp.z - 0.02 - THH }, true);   // snap concentric under cup
  it.setLinvel({ x: 0, y: 0, z: 0 }, true); it.setAngvel({ x: 0, y: 0, z: 0 }, true);
  const jd = RAPIER.JointData.fixed(
    { x: 0, y: 0, z: -0.02 - THH }, { w: 1, x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 }, { w: 1, x: 0, y: 0, z: 0 });
  welds.set(key, world.createImpulseJoint(jd, gr, it, true));
}

function doUnweld(g, slot) {
  const key = `${g}:${slot}`, j = welds.get(key);
  if (j) { world.removeImpulseJoint(j, true); welds.delete(key); }
}

function handle(req) {
  if (req.geom) { build(req.geom); return { ready: true }; }
  const dt = req.dt, bv = req.belt_v;
  for (let r = 0; r < (req.grippers || []).length; r++) setKin(grips[r], ...req.grippers[r]);
  for (const [slot, x, y, z] of req.totes || []) setKin(totes[slot], x, y, z);
  for (const [slot, x, y, z] of req.spawn || []) {
    const b = items[slot]; b.setTranslation({ x, y, z }, true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true); b.setAngvel({ x: 0, y: 0, z: 0 }, true);
    b.setRotation({ w: 1, x: 0, y: 0, z: 0 }, true);
  }
  for (const slot of req.park || []) {
    const b = items[slot]; b.setTranslation({ x: PARK[0] + 0.3 * slot, y: PARK[1], z: 0.05 }, true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true);
  }
  for (const [g, slot] of req.weld || []) doWeld(g, slot);
  for (const [g, slot] of req.unweld || []) doUnweld(g, slot);
  // belt = frictionless slab + scripted carry: belt items ride at belt speed (the Rapier
  // equivalent of MuJoCo's conveyor idiom; z is left to gravity/contact so they still settle)
  for (const slot of req.drag || []) {
    const b = items[slot], v = b.linvel();
    b.setLinvel({ x: bv, y: 0, z: v.z }, true);
  }
  const n = Math.max(1, Math.round(dt / world.integrationParameters.dt));
  world.integrationParameters.dt = dt / n;
  for (let s = 0; s < n; s++) world.step();

  const out = [];
  for (let i = 0; i < items.length; i++) {
    const b = items[i], t = b.translation(), q = b.rotation();
    out.push([i, r4(t.x), r4(t.y), r4(t.z), r4(q.w), r4(q.x), r4(q.y), r4(q.z)]);
  }
  return { items: out };
}
const r4 = v => Math.round(v * 1e4) / 1e4;

await RAPIER.init();
const rl = readline.createInterface({ input: process.stdin });
rl.on('line', (line) => {
  line = line.trim(); if (!line) return;
  let resp; try { resp = handle(JSON.parse(line)); } catch (e) { resp = { error: String(e) }; }
  process.stdout.write(JSON.stringify(resp) + '\n');
});
