# Solution
* What was your approach? (This is meant to be an open ended question. We're looking for your thought process and how you went about solving this problem.)
* What assumptions did you make?
* Did you consider any other approaches and why did you decide not to use them?

Because of the emphasis on the NEC rules, I broke the problem down roughly into two parts:
1. Segementing the room polygon into NEC compliant wall sections
2. Placing outlets appropriately along the wall sections according to NEC rules

The primary approach I used is "unwrapping" (projecting) the room polygon into a 1D line the length of all of the edges. Then you can project and unproject accordingly to operate within the much simpler 1D space. You want to project the doors/windows/kitchen bounds onto the 1D space, which makes segmenting the wall sections straightforward. Operations to support NEC rules for distance offsets of the outlets also straightforward, and opens up multiple possibilities.

There are other options to segment the walls, including polygon clipping and fancier approaches, but I liked how 1D projection set up an unified foundation into very simple solutions to both parts of the problem, and possibly related problems.

I generate outlet placements incrementally along the walls and then selected recommendations.

I opted for a simple incremental greedy approach to select outlet placement, which should get an optimally minimal number of outlets. Then I followed with a polynomial smoothing to better distribute the outlet locations. I think one of the outlets still ended up nearly flushed against a puck.

Two alternative outlet placement algorithms I considered were...
(A) Recursively subdividing the walls to find placement locations. This has the advantage of faster runtime potential over larger rooms, outlets options, etc., but felt a bit too necessary for the scope of this test.
(B) Stochastically trying and testing different placements. This has the advantage of decoupling the NEC rules/validation with the generation of the outlets, which I REALLY liked. For example, that enables expanded rulesets (based on interior design preferences, or international regulations) very easily. It's also fairly straightforward to implement, but ultimately, this approach seemed a bit too fancy and unnecessary given the scope.

I assumed valid JSONs formatted with valid floorplans with doors/windows/kitchen flushed against the room wall boundaries, while handling minor precision errors. I think windows, because they are floor length, could break the NEC code (if wider than 12 feet), so I also assumed they were of valid size. I also had to make assumptions regarding the keys ('pucks', 'generic_rooms', etc) -- but since it's an internal data format, I assume we can validate the floorplan data at a pre-processing step prior to running this algorithm.

# Implementation
* What was the runtime of your algorithm?
* What libraries did you add? What are they used for?
* How long did you spend on this? If you had more time, what would you add?
* What part are you most proud of and why?
* What did you find the most challenging?

It runs in about 1 second and is, in practice, limited primarily by (# outlet possibilities) x (# wall pucks) x (intersection checks). As the number of pucks scale up to the overall size of the room, I'd consider introducing more optimizations to reduce either of these numbers. (There are lots of low hanging fruits here.) With more time, I would clean up the code, write tests to validate the data input and output, and also include the aforementioned stochastic algorithm with additional rulesets geared towards aesthetics.

I used shapely for basic geometric operations (distance, intersection, etc) and numpy for simple smoothing-related math. I wrote a simple PIL-based visualizer to debug my progress. The solution should run on both Python 3.7 and 2.7, with shapely and numpy installed.

I took about 2 days which covers roughly 3-4ish hours per day, including breaks to walk and think, and this writeup. I generally liked my projection-based approach, and of course, debugging was the most time-consuming part.

# Other
* Anything else you'd like us to know (about your implementation or otherwise)?
* How did you find the test overall? Did you have any issues or have difficulties completing? If you have any suggestions on how we can improve the test, we'd love to hear them.

It was very fun! The diagrams were especially helpful :)