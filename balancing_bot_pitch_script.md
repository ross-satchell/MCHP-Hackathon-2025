# Balancing Bot Pitch Script

---

**[Begin]**
(SLIDE 1)
Imagine you’re at a Maker Fair—kids, teachers, and hobbyists all gathered around our classic Inverted Pendulum demo, when suddenly… the old kit dies.  
Too fragile, too complex, and a huge hassle to fix or replicate.
Add to that - These demos are not easy to get ahold of!
It's one of only a few that get ferried all over the globe. In fact, we've twice requested demos for shows or speaking engagements that never showed up in time.
That's not a sacrifice we can afford to miss out on, whether it's students, companies, or at tradeshows where we show off our tools.  
This was a wake-up call: If we’re going to ignite creativity and learning, ESPECIALLY as we expand in the academic and Maker world, we need a platform that’s robust, adaptable, and so easy anyone can use it.

**Problem:**  
The reality is, most demos are bulky, expensive, and require some expert knowledge.  
That’s a barrier for ambassadors in distant geographies, for students new to engineering, and for teachers with limited time or tools.  
We needed something better—something anyone could build, modify, and showcase, anywhere.

**Our Solution:**  
We have that now in the Workshop in a Box Pykit Ruler. And showcasing that, we have a new demo or module - a NEW balancing bot.  
Our team set out to leverage the same low-cost “Ruler” kit—the backbone of our new Workshop-in-a-Box.  
Because it’s affordable, available, it ships easily, you get to KEEP your own, and it was designed from the ground up to let beginners do more, faster — with complete documentation and ready-to-go modules.

The ruler is programmed in Python, using CircuitPy, so anyone with minimal programming experience can jump in and start experimenting.
But we didn’t stop at basics. We added 3D printed parts—you can get these anywhere, in bulk, or print yourself if you have the tools.
If you know some basic CAD, you can even make adjustments or add elements, to customize futher the the whole kit.  

(SLIDE ADVANCE - )

**Educational Value:**  
(SLIDE @)
This isn’t just another toy—it’s a hands-on introduction to real controls engineering, accessible for mechanical, electrical, and software engineers.  
It can anchor a whole curriculum, inspire hands-on courses beyond just the introductory workshop, and connect everything from lesson plans to independent research and student clubs.

**What Makes It Special:**  
Our Balancing Bot demo proves the kit’s power and flexibility.  
It’s impressive—just as dynamic as our old inverted pendulum—and it’s accessible.  
Even if this demo isn’t 100% perfect today, imagine what students could create on their own, with just these simple, well-documented tools.
This is their inspiration beyond just blinking an LED or learning how displays work - this is real engineering & controls, direct from Microchip.

**Vision:**  
We built this in under 24 hours—with parts and tools designed for the academic environment.  
If we can do that here, think what teachers and students could do if this mindset and kit reached classrooms and makerspaces across the globe.

**Next Steps:**
(SLIDE 3 - final)
We're just getting started—here’s how we plan to take this even further:

- First, we’ll record a video walk-through and demo—not just for show, but as a resource for course or module intros, so anyone can see it in action.
- Next, we’re formalizing the CAD files for all our 3D printed components, making them easily available for other educators and teams.
- We’re streamlining a complete build guide and modules—including a bill of materials, links for easy sourcing, and clear specs so anyone can replicate or adapt the build.
- And as a potential future upgrade, we’re looking to add simple Bluetooth communication—enabling wireless control, collaboration between bots, or remote classroom demos.

With each step, we’re lowering the barriers to hands-on engineering and multiplying the ways students and teams can learn, share, and innovate.

This is more than a hackathon project—it's showcasing an opportunity:  
Let’s bring the maker spirit to education.  
Let’s make complex engineering demos, approachable for everyone.  
Let’s encourage every student, teacher, and ambassador to push boundaries, invent, and collaborate—with these versatile tools we’ve created.

## Thank you

### Reality check

Building a piece of HW / Kit in a day is a SIZEABLE task...
We did order a bunch of pieces beforehand (wheels, motors, MC board, batteries, etc.)
We intended to but did NOT get a couple of inline fuses.
We also accidently fried the MC board we'd originally planned to use for this. So it wont be driving around today.

But CAD for the 3d printed parts was done day-of. And additional CAD files for mounts, boards, etc will replace the existing metal frame. (for battery holder, MC board mount, etc.)
Ideally this links up to border the ruler like additional "stairs" or "tiers" of modular pieces.
This is also part of the "customizable" nature of the project and board. If using a different MC board, it's very easy for someone with minor CAD experience to create a simple mount and 3d print it to slide onto the board, or stack on top of the existing part. (Keep the "Rails" and just model your own "mount" pice. It just slides along ruler, and with a small overlapping space like a half-lap joint, we add a hole for a screw to fasten 2 printed pieces to the ruler all together).

We have the code for motor control developed -
We've architected the right parts for PID control,
And there is core support for BT control over UART.

If we want to make this truly custom, we can create a simple MC board to sell along side the PyKit, to replace the Maker versions we used here today (and implement protections against the kind of mistakes we made ourselves) - such that the kit is more fault-proof, esp important when put into the hands of students, teachers, or ambassadors with less techincal experience.
The end goal would be to have customers/ambassadors only need to seperately order or have; a battery/power supply, small hobbiest motors, and wheels. the rest of the kit they can get from us, the PyKit ruler, and a small add-on board for MC / power connection pieces.  
