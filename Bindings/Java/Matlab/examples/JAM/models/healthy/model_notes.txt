The full_body_healthy_knee.osim model consists of a detailed multibody knee 
model with 6 degree of freedom (DOF) joints, articular contact, and ligaments. 
The knee bone, cartilage, and ligament geometries of this model were segmented 
from the MRI of a young healthy adult female. The Arnold generic lower 
extremity musculoskeletal model [1] was modified to remove several wrap objects
and adjust the maximum isometric strength of several lower limb muscles 
(done by Darryl Thelen... the details are buried in a SIMM file some where...).
The modified Arnold model was then scaled to the subject based on motion capture
markers and the detailed knee model was integrated. 

Versions:
---------

Lenhart 2015 [2]
- Model first introduced and validated against dynamic MRI. 

Smith 2019 [3]
- Added meniscus 

Current

Known Issues:
-------------


References:
-----------
[1] Arnold, E. M., Ward, S. R., Lieber, R. L., & Delp, S. L. (2010). A model of 
    the lower limb for analysis of human movement. Annals of biomedical 
    engineering, 38(2), 269-279.

[2] Lenhart, R. L., Kaiser, J., Smith, C. R., & Thelen, D. G. (2015). 
    Prediction and validation of load-dependent behavior of the tibiofemoral 
    and patellofemoral joints during movement. Annals of biomedical 
    engineering, 43(11), 2675-2685.

[3] Smith, C. R., Brandon, S. C., & Thelen, D. G. (2019). Can altered 
    neuromuscular coordination restore soft tissue loading patterns in anterior 
    cruciate ligament and menisci deficient knees during walking?. Journal of 
    biomechanics, 82, 124-133.