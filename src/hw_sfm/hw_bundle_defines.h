/*
* Some constants. Note that M_xx is not available everywhere.
* These constants are computed with the high precision calculator at
* http://keisan.casio.com/calculator set to 38 digits.
*/
#define HWMATH_PI         3.14159265358979323846264338327950288   // pi
#define HWMATH_PI_2       1.57079632679489661923132169163975144   // pi/2
#define HWMATH_PI_4       0.785398163397448309615660845819875721  // pi/4
#define HWMATH_1_PI       0.318309886183790671537767526745028724  // 1/pi
#define HWMATH_2_PI       0.636619772367581343075535053490057448  // 2/pi

/* Angle conversions macros, DEG <-> RAD. */
#define HWMATH_RAD2DEG(x) ((x) * (180.0 / HWMATH_PI))
#define HWMATH_DEG2RAD(x) ((x) * (HWMATH_PI / 180.0))