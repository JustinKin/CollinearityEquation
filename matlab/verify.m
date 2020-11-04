x1 = 1920;
y1 = 1080;

Cx1 = 1200;
Cy1 = 1200;

Fx1 = 800;
Fy1 = 800;

Tx1 = 50;
Ty1 = 50;
Tz1 = 50;

r0 = 1;
r1 = 0;
r2 = 0;
r3 = 0;
r4 = 1;
r5 = 0;
r6 = 0;
r7 = 0;
r8 = 1;

x2 = 2300;
y2 = 1440;

Cx2 = 1100;
Cy2 = 1100;

Fx2 = 800;
Fy2 = 800;

Tx2 = 60;
Ty2 = 60;
Tz2 = 60;

a1 = x1- Cx1;
b1 = y1 - Cy1;
a2 = x2- Cx2;
b2 = y2 - Cy2;

a00 = (a1 * r6) - Fx1 * r0 ;
a01 = (a1 * r7) - Fx1 * r1 ;
a02 = (a1 * r8) - Fx1 * r2 ;
a10 = (b1 * r6) - Fy1 * r3 ;
a11 = (b1 * r7) - Fy1 * r4 ;
a12 = (b1 * r8) - Fy1 * r5 ;

a20 = (a2 * r6) - Fx2 * r0 ;
a21 = (a2 * r7) - Fx2 * r1 ;
a22 = (a2 * r8) - Fx2 * r2 ;
a30 = (b2 * r6) - Fy2 * r3 ;
a31 = (b2 * r7) - Fy2 * r4 ;
a32 = (b2 * r8) - Fy2 * r5 ;

A = [ a00,a01,a02;
      a10,a11,a12;
      a20,a21,a22;
      a30,a31,a32];
  
b01 = Fx1 * Tx1 - a1 * Tz1;
b02 = Fy1 * Ty1 - b1 * Tz1;
b03 = Fx2 * Tx2 - a2 * Tz2;
b04 = Fy2 * Ty2 - b2 * Tz2;

B = [ b01; b02; b03 ;b04 ];
AT = A';
ATA = AT * A;
ATB = AT * B;
X = (A' * A )\(A' * B);