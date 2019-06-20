# Fixing missing polygons

(1) Download Paraview and open .vtp file.

(3) Use hotkey (S) to switch on polygon selection, select incorrect polygons.

(4) Open Paraview Selection Display Inspector, select the Point Labels and ID. The selected Polygons will now show their coordinates.

(5) Open the .vtp file in a text editor and search for the three numbers seen in Paraview. Switch the 1st and 3rd numbers.

36 120 210 -> 210 10 210

There may be one or two sets of with these three numbers, change all.

36 120 210 -> 210 120 210
210 36 120 -> 120 36 210
