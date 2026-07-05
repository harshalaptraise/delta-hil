# First Principles in Motion

There is a particular kind of proof you can only feel with your finger on a variable.

Late in a long day, I had a real Beckhoff soft-PLC running a cyclic task, and on the screen a two-robot delta cell picking tortillas off a moving belt and placing them into moving totes. I opened a watch window, found a boolean called `enable`, and forced it to `FALSE`. The whole cell froze mid-reach. I forced it back to `TRUE`. It resumed exactly where it had stopped. Nothing in the picture was real — no robot, no belt, no tortilla — but the thing deciding when they moved was a genuine industrial controller, and I could stop the world by touching it.

That is the moment digital-twin work is actually about. Everything before it is plumbing.

I built this over a single day by directing an AI collaborator — I set the constraints and the taste, it wrote the code and ran it on the rig. What follows is less about the robot and more about the *method*, because the method is the transferable part.

> *How do you get a real-time industrial controller to believe it is running a physical cell that does not exist — and do it without quietly fooling yourself?*

<!-- VIDEO: drop the final HD mp4 here -->
<video controls width="100%" style="border-radius:6px">
  <source src="REPLACE_WITH_VIDEO_URL.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Principles before code

We did not start by writing code. We started by arguing about what could not be violated.

Before the first line, we ran a short pre-flight — a calibration. Name the domain's first principles: the handful of irreducible truths the whole thing rests on, each a one-line invariant with a one-line justification. Not best practices, not conventions — the things that, if broken, mean the result is wrong no matter how convincing it looks. For this cell they were things like: *the controller decides and the plant only senses and actuates*; *a grasp is real only if the tool coincides with the part in position and velocity*; *tracking error grows with loop latency*; *every part is conserved — none teleports or vanishes*.

Then I approved them, deliberately, as a constitution for the session. And only from those approved principles did we derive the evals — the concrete, checkable pass/fail tests that would decide whether "done" was actually true. No reach command outside the measured envelope. A grasp accepted only on position-and-velocity coincidence. A ledger where parts in equals parts picked plus parts passed. A control loop under ten milliseconds. Each eval traced back to a named principle; nothing was on the list because it felt right.

This sounds like ceremony. It is the opposite. **The hard part of working with an AI is not getting it to write code — it is defining "correct" precisely enough that neither of you can drift.** A model will cheerfully optimize toward whatever target it infers, and if the target is "make it look good," it will make it look good and hide the seams. Drawing the measuring stick *before* the work — agreeing the invariants and the pass/fail bar first — is what turns a generative tool into an engineering collaborator. It gives the model a fixed thing to be right about, and it gives you a fixed thing to check.

Much later in that same day, when we quietly widened a reach window or changed a belt speed, we re-ran those same evals every time. The measuring stick never moved. That is the only reason the changes stayed honest, and it is the part of this I would carry to any project, robot or not.

## The journey, in feedback

I did not hand over a spec. I handed over a direction — *a high-fidelity delta robot talking to a PLC in a real-time simulation* — and then I steered, one render at a time.

The loop was relentless and it was mine to close: the model would push code, I would run it on the rig, paste back a screenshot or a GIF, and say what was wrong. The boxes overlapped. The tortillas came too fast. Robot B stood idle while a reachable tortilla sailed past. The place motion "went in" on the first cycle and hovered absurdly on the second. Each of these was a single, specific, checkable complaint, and each one forced a specific fix.

This is worth stating plainly, because it is the whole discipline: **you cannot supervise what you cannot see.** A digital twin is a supervision instrument before it is anything else. The GIF was not decoration; it was the measuring stick. Every time I could name the flaw in one sentence, the flaw got fixed. Every time I waved at "make it better," nothing did.

## Mock first, then fill it in

We never built the hard thing first. We built a *fake* of it, and then replaced the fake one layer at a time.

Before a single line touched TwinCAT, there was a `MockCellController` in Python — the exact controller the PLC would later become — driving a pure-Python plant with no graphics at all. That let us settle the behaviour on a laptop, deterministically, in milliseconds: who picks what, when a robot commits to a tote, why it must never drop a tortilla it has already grabbed. Only once that mock was the "golden reference" did we translate it, line for line, into a TwinCAT function block.

The same trick ran top to bottom. The plant was a headless integrator first, an Isaac Sim render second. The transport was a laptop-tested tag map first, live ADS second. The visualization was one robot at a fixed point before it was two robots on streaming belts.

There is an old engineering instinct here: **a mock is not a shortcut, it is a contract.** When the real PLC finally drove the cell and it looked *identical* to the mock, that was not luck — it was because the mock had defined, in advance, exactly what "correct" meant. The one time the live PLC diverged, we had a golden reference to diff against, and the divergence took minutes to find instead of hours.

The plant, crucially, never learned any control logic. It streams parts and totes, executes whatever tool-centre-point the controller commands, decides whether a grasp actually coincided in position *and* velocity, and conserves a ledger of every tortilla. It senses and it actuates. It does not think. Keeping that boundary religious is what lets the same plant run under a mock controller on a laptop and a real PLC on the rig without changing a line.

## The architecture, and the clock

The PLC program is deliberately small. Each robot is one function block — a state machine that claims a part, tracks its live position so the tool velocity matches the belt, grips only after a short lock, carries, commits to a single tote, tracks *that*, descends, releases, and comes home. An upstream robot takes its share and leaves the rest; a downstream robot is greedy and catches whatever slips past. A supervisor variable can freeze the whole thing. That is the entire brain: two instances of one block, a few global arrays, and a lot of restraint.

The interesting part is time. A PLC is a discrete controller running a hard real-time task. A simulation has no clock at all — it advances only when you hand it a `dt`. Get that `dt` wrong and everything downstream is a lie: our first live attempt had the robot chasing a tortilla it could never grab, because the sim was advancing a fixed step per scan while the PLC's lock timer counted real milliseconds. The part left the pick window before the grip completed.

The fix was to stop the sim from having opinions about time, and let it read the controller's clock:

```
dt = (plc_time_ns - prev_plc_time_ns) / 1e9
if dt > 0:
    plant.step(dt)      # advance ONLY when the PLC clock actually ticked
```

The PLC publishes its own nanosecond clock every cycle; the sim integrates by exactly that elapsed time, one plant step per sample. One authoritative clock — the controller's — drives both sides, and it self-corrects for the jitter in when the ADS packets happen to land. The measured round-trip settled at about **2 ms, with a quarter of a millisecond of jitter**. The plant became a faithful follower of the controller's timeline.

That number is not a vanity metric — and it is the one I would put in front of anyone asking whether this transfers to a real robot. Loop latency *is* the tracking error budget: a part moving at a fifth of a metre per second, sampled two milliseconds late, is off by less than half a millimetre — inside grip tolerance with room to spare. The sub-millisecond jitter is the determinism real-time control actually requires; a loop that wanders is a loop you cannot trust to grip a moving object. This is precisely what has to survive the jump to hardware. A physical cell runs the *same* controller over a hard EtherCAT fieldbus, whose latency and jitter are tighter and more deterministic than a polled ADS link by construction. So the timing envelope the controller was validated in is one the real machine will meet or beat — the tracking that held against the twin will not fall apart against the robot for want of loop budget. Two milliseconds with a quarter of jitter is a green light. Ten with five would have been a warning to redesign the transport before ordering a single motor.

That is the sentence I would attribute the whole project to: *the clock is the contract.* Positions, velocities, grip states — those are the easy signals. The one that has to be right, and is the one everyone skips, is time.

## A continuous plant and a discrete mind

Near the end, the sharpest exchange of the day was not about code. It was about whether we were even modelling the right thing.

The obvious "correct" answer to synchronization is a hard lockstep — an EtherCAT distributed clock, both sides stepping in sub-microsecond agreement. I was ready to chase it. The pushback stopped me: a real plant is *analogue*. It integrates itself, continuously, with no clock and no ticks. Encasing a plant in a hard master clock is not more faithful to reality — it is imposing digital determinism on the one thing that, physically, has none. What a real control system actually *is* is a continuous plant sampled by a discrete controller, with real latency in the loop.

Seen that way, the humble ADS-polling arrangement is not a compromise at all. It is the honest topology of sampled-data control: the PLC samples a continuous plant, holds its output, and the plant integrates in between. Chasing lockstep would have been chasing a tighter *sampling interval*, not a truer *physics*. So we did not sub-step the integrator either — for a kinematic plant, one Euler step per sample carries about a millimetre of error, well under grip tolerance, and sub-stepping would only have smuggled a finer internal clock back into the plant we were trying to keep continuous.

The lesson generalizes past this cell. **The fidelity that matters is rarely the fidelity people optimize.** It is easy to spend a week tightening a clock and never notice you were modelling the wrong system.

And, in the spirit of not fooling yourself: it is not perfect. Robot B still lets a tortilla or two through when it is holding one and waiting for a tote to come round. That is a deliberate consequence of a rule we chose — *never abandon a part you have picked* — and I would rather have that honest limitation than a demo that hides it. It ran. Both robots picked. The belts moved. The PLC was in charge. Above all, it tracked.

## Is this actually new?

I wanted an honest answer, not a press release, so we searched.

The individual ingredients all exist in public, in isolation. Isaac Sim ships a conveyor utility and pick-and-place tutorials — but from static, known poses, not a tool velocity-matching a moving product on the fly [1]. Vendors have used Isaac Sim to *train grippers* for items on fast conveyors — but that is synthetic-data perception, not tracking control [2]. A real Beckhoff PLC can already talk to Isaac Sim over ADS through the open-source Loupe bridge — so "PLC-in-the-loop with Isaac" is emphatically **not** new [3]. Conveyor tracking itself is decades old and mature — in RoboDK, in FANUC line tracking, on real robots — just not inside Isaac Sim [4]. And running a *closed-loop delta robot* in Isaac Sim is genuinely awkward, because PhysX articulations do not natively support closed kinematic chains, with no public IRB-360 pick-and-place example to be found [5].

So the defensible claim is a narrow, scoped one, and I will state it as such: **closing a real TwinCAT PLC around Isaac Sim to drive a delta robot performing velocity-matched conveyor-tracking pick-and-place appears to have no public prior example** — even though every component of it exists separately. The novelty is the *intersection*, not any single piece. Honesty about the pieces is what makes the intersection believable.

## Taking it to the bench

A twin earns its keep only if it is the on-ramp to hardware. So: what stays, what drops, what arrives.

**What stays.** The whole PLC program — unchanged. That is the entire point of hardware-in-the-loop: the controller that ran the simulated cell is the controller that runs the real one. The tag map stays. The sampled-data discipline stays. Isaac Sim stays too, but its role inverts — from *the plant* to a *shadow* running beside the real line for monitoring and what-if.

**What drops.** The kinematic shortcut. In the twin the plant *decides* whether a grasp coincided; on the bench, physics and a real vacuum gripper decide, and cameras and photo-eyes report it. The soft, best-effort ADS time base drops in favour of a hard real-time fieldbus. And the Python integrator that stood in for the plant drops out entirely.

**What arrives.** The physical layer, and it is the expensive half: two real IRB 360s and their drives, an EtherCAT bus with distributed-clock sync replacing the polled ADS loop, a vision system to register incoming parts and their positions, belt encoders so the tracking rides a real conveyor, safety interlocks, and the calibration to make the robot's frame and the camera's frame agree to a fraction of a millimetre. The controller believes the same things; it simply gets its senses from silicon and steel instead of from a simulation.

That asymmetry — controller unchanged, senses swapped — is the reason to build the twin at all. You debug the mind in software, where mistakes cost seconds, so that on the bench you are only debugging the body.

The clock was the contract in the twin, and it will be the contract on the bench. Only the hands change.

---

*Built in a day, directed by hand, one render at a time. The code is a private repo; the method is the point.*

**References**

1. NVIDIA Isaac Sim — Conveyor Belt Utility & pick-and-place tutorials. docs.isaacsim.omniverse.nvidia.com (conveyor utility; pick from static poses).
2. Soft Robotics / Oxipital, *mGripAI trained in NVIDIA Isaac Sim* — synthetic-data gripper training for conveyor items. blogs.nvidia.com / therobotreport.com.
3. Loupe, *Omniverse Beckhoff Bridge Extension* — open-source TwinCAT↔Isaac Sim over ADS. github.com/loupeteam/Omniverse_Beckhoff_Bridge_Extension.
4. Visual conveyor tracking / pick-on-the-fly — RoboDK conveyor tracking; FANUC line tracking; IEEE, *Visual conveyor tracking for pick-on-the-fly robot motion control*.
5. Isaac Sim / PhysX closed-loop articulations — closed kinematic loops need Guide-Joint / exclude-from-articulation workarounds; no public IRB 360 / FlexPicker pick-and-place example found.
