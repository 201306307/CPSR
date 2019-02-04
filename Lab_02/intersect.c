#include <stdbool.h>

int intersect(float p0_x, float p1_x, float p2_x, float p3_x, float p0_y, float p1_y, float p2_y, float p3_y, float *point) {
 
	 float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
	 s10_x = p1_x - p0_x;
	 s10_y = p1_y - p0_y;
	 s32_x = p3_x - p2_x;
	 s32_y = p3_y - p2_y;

	 denom = s10_x * s32_y - s32_x * s10_y;
	 if (denom == 0){
	    return 0; // Collinear
	 }
	 bool denomPositive;
	 denomPositive = denom > 0;

	 s02_x = p0_x - p2_x;
	 s02_y = p0_y - p2_y;
	 s_numer = s10_x * s02_y - s10_y * s02_x;
	 if ((s_numer < 0) == denomPositive){
	    return 1; // No collision
	 }

	 t_numer = s32_x * s02_y - s32_y * s02_x;
	 if ((t_numer < 0) == denomPositive){
	    return 1; // No collision
	 }

	 if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive)){
	    return 1; // No collision
	 }
	 // Collision detected
	 t = t_numer / denom;
	 point[0] = (p0_x + (t * s10_x));
	 point[1] = (p0_y + (t * s10_y));


	 return 2;
 }