Name: Neil Nie

Individual Team. (1) late day used.

## EdgeCollapse (8pts)


After testing, I believe this is working as intended, and handles the edge cases outlined
in the instructions.


## EdgeFlip (8pts)


After testing, I believe this is working as intended, and handles the edge cases outlined
in the instructions. Specifically, this is tested on polygon edge flip and the function
appears to be working.

## EdgeSplit (8pts)


After testing, I believe this is working as intended, and handles the edge cases outlined
in the instructions.

## FaceBevel (8pts)


After testing, I believe this is working as intended, and handles the edge cases outlined
in the instructions.

## Triangulation (10pts)


After testing, I believe this is working as intended, and handles the edge case cases outlined
in the instructions.


## LinearSubdivision (10pts)

After testing, I believe this is working as intended, and handles the edge cases outlined
in the instructions.


## CatmullClarkSubdivision (10pts)

After testing, I believe this is working as intended, and handles the edge cases outlined
in the instructions.

## Simplification (20pts)

Simplification is working on the test meshes. One small issue is that faces
might be accidentally flipped during collapse. This is not correct and violates
the manifoldness of the mesh. Some attempts are made to remedy this by checking
the face normal before and after the collapse. But after all, simplification is
working extremely close to as intended.

## Model Something!! (10pts)

A new mesh called `model.dae`. The model aims to represent the 
mask worn by Darth Vader from Star Wars. To create this mesh, multiple local and
global operations are used, such as triangulate, linear sub-division, and face bevel.

## Extra Credit

Two extra credit method are implemented
- `erase_edge`
- `erase_vertex`
