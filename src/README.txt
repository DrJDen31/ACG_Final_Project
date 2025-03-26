HOMEWORK 3: RAY TRACING, RADIOSITY, & PHOTON MAPPING

NAME:  Jaden Tompkins



ESTIMATE OF # OF HOURS SPENT ON THIS ASSIGNMENT:  35 hours



COLLABORATORS AND OTHER RESOURCES: List the names of everyone you
talked to about this assignment and all of the resources (books,
online reference material, etc.) you consulted in completing this
assignment.

None

Remember: Your implementation for this assignment must be done on your
own, as described in "Academic Integrity for Homework" handout.



OPERATING SYSTEM & VERSION & GRAPHICS CARD:   Windows 11 Home version 23H2, 
					     NVIDIA GeForce RTX 3060 Laptop GPU 
						running NVIDIA Studio Driver version 566.14



SELF GRADING TOTAL:  [ 16 / 20 ]


< Please insert notes on the implementation, known bugs, extra credit
in each section. >



2 PROGRESS POSTS [ 5 / 5 ] Posted on Submitty Discussion Forum on the
dates specified on the calendar.  Includes short description of
status, and at least one image.  Reasonable progress has been made.


SPHERE INTERSECTIONS, SHADOWS, & REFLECTION [ 2 / 2 ]
  ./render -size 200 200 -input reflective_spheres.obj 
  ./render -size 200 200 -input reflective_spheres.obj -num_bounces 1
  ./render -size 200 200 -input reflective_spheres.obj -num_bounces 3 -num_shadow_samples 1 

Sphere intersection implemented as described in class, by subbing one equation into the other and finding the smallest valid "t" value. 
Shadows implemented by casting a trace to each light and adding the value from that light if it isn't blocked.
Reflections implemented by recursively calling "TraceRay" in the reflective direction, decrementing "bounce_count" to track how many levels deep.


RAY TREE VISUALIZATION OF SHADOW & REFLECTIVE RAYS [ 1 / 1 ]


DISTRIBUTION RAY TRACING: SOFT SHADOWS & ANTIALIASING [ 2 / 2 ]
  ./render -size 200 200 -input textured_plane_reflective_sphere.obj -num_bounces 1 -num_shadow_samples 1
  ./render -size 200 200 -input textured_plane_reflective_sphere.obj -num_bounces 1 -num_shadow_samples 4
  ./render -size 200 200 -input textured_plane_reflective_sphere.obj -num_bounces 1 -num_shadow_samples 9 -num_antialias_samples 9

Soft shadows expand upon the normal shadows by using the center of the light source for the first sample, and then a random point on the light source for each additional sample. 
Anti-aliasing was implemented by doing multiple samples per pixel, the first through the center, then the rest with a random offset within the pixel. For both, the provided, global rand() function was used, which gives a value 0-1.

EXTRA CREDIT: SAMPLING [ 0 ]
1 point for stratified sampling of pixel in image plane
1 point for stratified sampling of soft shadows
includes discussion of performance/quality


OTHER DISTRIBUTION RAY TRACING EXTRA CREDIT [ 0 ]
glossy surfaces, motion blur, or depth of field, etc.


BASIC FORM FACTOR COMPUTATION [ 1 / 2 ]
Description of method in README.txt.  
  ./render -size 200 200 -input cornell_box.obj

Form factor computation is done by implementing the equation from the slides. For each pair of faces, a selection of random samples are cast from one to the other. If the cast succeeds, use it to calculate visibility. Either way, add the sample's contribution to the form factor using the given equation. At the end, multiply by the inverse of the area and the visibility.
 
Note: My solution is close enough for the submitty autograder, but currently does seem to not be 100% correct, a few squares are weighted stronger than they should be.


RADIOSITY SOLVER [ 1 / 3 ]
May be iterative (solution fades in) or done by inverting the form
factor matrix.

Radiosity is done by finding the face with the most undistributed energy. This is then distributed to each face proportionally to the form factor relating the two faces (keeping direction in mind).

Note: My solution is close enough for the submitty autograder, but currently does seem to not be 100% correct. This is likely partly due to the incorrect form factors, but it also appears to improperly handle faces of different sides. For example, in the diffuse sphere scenes, the sphere faces absorb much less of the light than expected and appear almost black. I assume to fix this, I need to account for the area of a face when distributing light to it, but haven't found the correct way to implement it. This could be the common bug of not normalizing the form factors properly for different size faces.


FORM FACTORS WITH VISIBILITY / OCCLUSION RAY CASTING [ 1 / 1 ]
  ./render -size 300 150 -input l.obj 
  ./render -size 300 150 -input l.obj -num_form_factor_samples 100
  ./render -size 300 150 -input l.obj -num_shadow_samples 1 
  ./render -size 300 150 -input l.obj -num_form_factor_samples 10 -num_shadow_samples 1 
  ./render -size 200 200 -input cornell_box_diffuse_sphere.obj -sphere_rasterization 16 12
  ./render -size 200 200 -input cornell_box_diffuse_sphere.obj -sphere_rasterization 16 12 -num_shadow_samples 1

Visibility is described in the form factor calculation section.

Note: My solution seems to handle visibility properly, however it just has the afformentioned issues. Therefore, while these given inputs don't appear 100% correct, I'm attributing it to the above and taking off points there rather than here, as the expected effect of the visibility is seen. I would expect to not have to adjust this specifically if the other issues are fixed.


RADIOSITY EXTRA CREDIT [ 0 ]
1 point for ambient term in radiosity
1-2 points for new test scene or visualization
1 point for writing the ray traced image to a file
1-3 points extra credit for performance improvements
1-3 points for other ray tracing effects
1-3 points for gradient or discontinuity meshing in radiosity 


PHOTON DISTRIBUTION [ 2 / 2 ]
Shoots photons into the scene and the visualization looks reasonable
(the heart shape can be seen in the ring).
  ./render -size 200 200 -input reflective_ring.obj -num_photons_to_shoot 10000 -num_bounces 2 -num_shadow_samples 10
  ./render -size 200 200 -input reflective_ring.obj -num_photons_to_shoot 500000 -num_bounces 2 -num_shadow_samples 10 -num_antialias_samples 4

For each photon, make the given trace into the scene. Don't store the first hit. If beyond the first hit, store photons that hit diffuse surfaces. Either way, if a reflective surface is hit, calculate the new direction and recursively trace the next photon. Else, if a diffuse surface is hit, determine a random new direction and recursively trace the photon. The next photon is weighted by the reflective/diffuse color of the face's material. This continues until the energy of the photon dies out.


RAY TRACING WITH PHOTONS [ 1 / 2 ]
Searching for photons in the kdtree to use in ray tracing.  The
caustic ring is visible, and there are no significant artifacts in
illumination.

The gathering of photons is done by collecting photons within a bounding box. Then, the photons need to be checked to see if they are actually in the desired radius and if they came from a reasonable direction. The valid ones are stored in a set. If there are enough of them, the first (closest) num_photons_to_collect photons are used, with their energies summed. This is scaled relative to the radius of the circle containing them (i.e. the distance from the point to the furthest of these photons). 

Note: This doesn't work with the cornell_box_reflective_sphere example, with cubes of uniform color being seen. Additionally, the check of if a photon is valid when gathering is not very extensive (distance and dot product of direction).


PHOTON MAPPING MATCHES RADIOSITY [ 0 ]
The intensity and color bleeding of photon mapping for indirect
illumination are correctly weighted and closely matches the results
from radiosity.  2 points extra credit.
  ./render -size 200 200 -input cornell_box_diffuse_sphere.obj -num_photons_to_shoot 500000 -num_shadow_samples 500 -num_photons_to_collect 500 



OTHER EXTRA CREDIT [ 0 ]
1-2 points for new test scene or visualization
1 point for writing the ray traced image to a file
1-3 points extra credit for performance improvements
2-5 points for irradiance caching

<Insert instructions for use and test cases and sample output as appropriate.>

None.



KNOWN BUGS IN YOUR CODE
Please be concise!

No major bugs that I have seen aside from the improper radiosity behavior. On Submitty, it seems like the window freezes or times out while calculating form factors. I would guess this means my implementation is slow enough where Submitty will try to use the form factors before they finish calculating and the window will react with "not responding".

Additionally, photon mapping with the cornell box and spheres isn't quite there. It seems to create cubes of uniform color, rather than a finer gradient of color.
