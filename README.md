# Outlet Placer

One of the things we do at Social Construct is to programmatically create building details based on rough sketches of floor plans. In the following problem, you'll place outlets in a sample studio apartment according to rules of the 2017 NEC electrical code.

## Some Background
All of Social Construct's cables, pipes, and assorted in-wall infrastructure lives under the floor as part of our [raised-floor system](https://techcrunch.com/2020/07/14/social-constructs-computer-optimized-buildings-could-shake-construction-industrys-foundations/). Floorboards sit on top of support structures called "pucks", which come in 3 sizes: 4"x4", 4"x2", and 2"x2".

## Rules of Outlet Placement
Here are basic rules of electrical outlet placement per the NEC rules:
* The maximum distance to a receptacle (outlet) along a wall is 6 feet (72 inches)
* A wall is defined as any space longer than 2 feet (24 inches)
    * Wall space includes the space measured around corners
    * The wall space continues unless broken by a doorway (aka, doors **do not** count towards the length of a wall segment)
    * The space occupied by windows counts as wall space (aka, windows **do** count towards the length of a wall segment)
* This [illustration](https://www.naffainc.com/x/CB2/Elect/EImages/outletsneeded.gif) may be helpful in summarizing the above rules

# Other Rules
* For this example problem, kitchens do not count as wall space (they have their own set of rules, which we'll ignore for simplicity)
* You can ignore bathrooms and closets
* All windows are floor-to-ceiling windows and cannot have outlets in front of them (even though they contribute towards total wall length)
* SoCo outlets are 2" deep and 4" wide
* Outlets must go in-between pucks (the feet / supports of the SoCo raised floor system)
![Allowable Configuration]()

## Files and Folders
`json/studio_info.json` and `json/floor_info.json` are the two main files you need to get this project started.

* `json/studion_info.json` contains the coordinates in WCS for all the rooms, windows, and doors of this sample studio apartment.
* `json/floor_info.json` contains the coordinates in WCS for all floors and pucks of the SoCo flooring system for this sample studio.

To simplify things, both files store coordinates in **decimal inches**.

## Submittal
Please submit the following:
* outlets.json - a file containing the coordinates in WCS for outlet locations
* outlet_placer.py - with your code
* solution.txt - a brief explanation of what you did and why

We mainly want to see your code, and strongly prefer Python. If you have a strong preference for another language, you may submit your solution that way, but please include an explanation as to why.

## Helpful Sources
You can use whatever open source libraries you think might be helpful in solving the problem. Some of our favorites include:

* [Shapely](https://pypi.org/project/Shapely/) is a helpful Python library for manipulation and analysis of planar geometric objects.

* [Pulp](https://pypi.org/project/PuLP/) and [Google OR Tools](https://developers.google.com/optimization) include powerful optimization tools

* Autodesk provides easy [online viewers](https://viewer.autodesk.com/) for various CAD files

* Receptacle [spacing requirements](https://www.ecmag.com/section/codes-standards/article-210-branch-circuits-6) per the 2017 NEC electric code, if you're curious about the rules

## Other
Please don't spend too much time on this. It shouldn't take more than an hour or two. We want to see how you think, and how you approach problems. We're looking for functional, readable code rather than perfection.