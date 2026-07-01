// =============================================================================
// catom3d_core.hpp
// Webots-independent core for the Catom3D trace player.
//
// Shared by the Webots controller (catom3d_controller.cpp) and the offline
// verifier (verify_trace.cpp) so both reproduce moves with identical geometry.
//
//  1. Math types  (Vec3, Mat4)   — port of VisibleSim matrix44 / vector3D
//  2. Geometry constants         — connector positions, orientations, face tables
//  3. FCC lattice helpers        — grid<->world, neighbor tables
//  4. Motion rule engine         — 120 links built at startup
//  5. Rotation animator          — port of Catoms3DRotation::nextStep
//  6. Trace player helpers       — load trace.csv, reproduce each roll
//
// All functions/globals are static (internal linkage): include in exactly one
// translation unit per executable.
// =============================================================================

#ifndef CATOM3D_CORE_HPP_
#define CATOM3D_CORE_HPP_

#include <cmath>
#include <cstring>
#include <cassert>
#include <array>
#include <vector>
#include <bitset>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <set>
#include <algorithm>
#include <iostream>
#include <limits>
#include <functional>
#include <tuple>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880
#endif

// =============================================================================
// SECTION 1 — MATH TYPES
// =============================================================================

struct Vec3 {
    double v[4];
    Vec3() : v{0,0,0,0} {}
    Vec3(double x,double y,double z,double w=0.0):v{x,y,z,w}{}
    double  operator[](int i) const { return v[i]; }
    double& operator[](int i)       { return v[i]; }
    double norm() const { return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
    Vec3 normalized() const { double n=norm(); return n>1e-12?Vec3(v[0]/n,v[1]/n,v[2]/n,v[3]):*this; }
    Vec3 operator+(const Vec3& o) const { return {v[0]+o.v[0],v[1]+o.v[1],v[2]+o.v[2],v[3]+o.v[3]}; }
    Vec3 operator-(const Vec3& o) const { return {v[0]-o.v[0],v[1]-o.v[1],v[2]-o.v[2],v[3]-o.v[3]}; }
    Vec3 operator*(double s)      const { return {v[0]*s,v[1]*s,v[2]*s,v[3]*s}; }
    Vec3 operator-()              const { return {-v[0],-v[1],-v[2],-v[3]}; }
    double dot(const Vec3& o)     const { return v[0]*o.v[0]+v[1]*o.v[1]+v[2]*o.v[2]; }
    Vec3 cross(const Vec3& o)     const {
        return {v[1]*o.v[2]-v[2]*o.v[1],v[2]*o.v[0]-v[0]*o.v[2],v[0]*o.v[1]-v[1]*o.v[0],0.0};
    }
};
inline Vec3 operator*(double s,const Vec3& u){return u*s;}

struct Mat4 {
    double m[16];
    Mat4(){ identity(); }
    void identity(){ memset(m,0,sizeof(m)); m[0]=m[5]=m[10]=m[15]=1.0; }

    void setTranslation(double x,double y,double z){
        identity(); m[3]=x; m[7]=y; m[11]=z; m[15]=1.0;
    }
    void setTranslation(const Vec3& t){ setTranslation(t[0],t[1],t[2]); }

    // Rotation around axes — DEGREES (matches VisibleSim)
    void setRotationX(double a){ identity(); double c=cos(a*M_PI/180.),s=sin(a*M_PI/180.); m[5]=c;m[6]=-s;m[9]=s;m[10]=c; }
    void setRotationY(double a){ identity(); double c=cos(a*M_PI/180.),s=sin(a*M_PI/180.); m[0]=c;m[2]=s;m[8]=-s;m[10]=c; }
    void setRotationZ(double a){ identity(); double c=cos(a*M_PI/180.),s=sin(a*M_PI/180.); m[0]=c;m[1]=-s;m[4]=s;m[5]=c; }

    // Rodrigues — DEGREES
    void setRotation(double a,const Vec3& V){
        double c=cos(a*M_PI/180.),s=sin(a*M_PI/180.),c1=1.-c;
        m[0]=V[0]*V[0]*c1+c;      m[1]=V[0]*V[1]*c1-V[2]*s; m[2]=V[0]*V[2]*c1+V[1]*s; m[3]=0;
        m[4]=V[0]*V[1]*c1+V[2]*s; m[5]=V[1]*V[1]*c1+c;      m[6]=V[1]*V[2]*c1-V[0]*s; m[7]=0;
        m[8]=V[0]*V[2]*c1-V[1]*s; m[9]=V[1]*V[2]*c1+V[0]*s; m[10]=V[2]*V[2]*c1+c;     m[11]=0;
        m[12]=0;m[13]=0;m[14]=0;m[15]=1;
    }

    Mat4 operator*(const Mat4& p) const {
        Mat4 r; r.identity();
        for(int l=0;l<4;l++) for(int c=0;c<4;c++){
            r.m[l*4+c]=0;
            for(int i=0;i<4;i++) r.m[l*4+c]+=m[l*4+i]*p.m[i*4+c];
        }
        return r;
    }
    Vec3 operator*(const Vec3& p) const {
        double x=0,y=0,z=0,w=0;
        for(int i=0;i<4;i++){x+=m[i]*p.v[i];y+=m[i+4]*p.v[i];z+=m[i+8]*p.v[i];w+=m[i+12]*p.v[i];}
        return {x,y,z,w};
    }
    Vec3 getPosition() const { return {m[3],m[7],m[11],1.0}; }
    void inverse(Mat4& inv) const;
};

inline double det33(double a,double b,double c,double d,double e,double f,double g,double h,double i){
    return a*(e*i-h*f)-d*(b*i-h*c)+g*(b*f-e*c);
}
inline void Mat4::inverse(Mat4& inv) const {
    double det=m[0]*det33(m[5],m[6],m[7],m[9],m[10],m[11],m[13],m[14],m[15])
              -m[4]*det33(m[1],m[2],m[3],m[9],m[10],m[11],m[13],m[14],m[15])
              +m[8]*det33(m[1],m[2],m[3],m[5],m[6],m[7],m[13],m[14],m[15])
              -m[12]*det33(m[1],m[2],m[3],m[5],m[6],m[7],m[9],m[10],m[11]);
    inv.m[0]= det33(m[5],m[6],m[7],m[9],m[10],m[11],m[13],m[14],m[15])/det;
    inv.m[4]=-det33(m[4],m[6],m[7],m[8],m[10],m[11],m[12],m[14],m[15])/det;
    inv.m[8]= det33(m[4],m[5],m[7],m[8],m[9], m[11],m[12],m[13],m[15])/det;
    inv.m[12]=-det33(m[4],m[5],m[6],m[8],m[9],m[10],m[12],m[13],m[14])/det;
    inv.m[1]=-det33(m[1],m[2],m[3],m[9],m[10],m[11],m[13],m[14],m[15])/det;
    inv.m[5]= det33(m[0],m[2],m[3],m[8],m[10],m[11],m[12],m[14],m[15])/det;
    inv.m[9]=-det33(m[0],m[1],m[3],m[8],m[9], m[11],m[12],m[13],m[15])/det;
    inv.m[13]= det33(m[0],m[1],m[2],m[8],m[9], m[10],m[12],m[13],m[14])/det;
    inv.m[2]= det33(m[1],m[2],m[3],m[5],m[6],m[7],m[13],m[14],m[15])/det;
    inv.m[6]=-det33(m[0],m[2],m[3],m[4],m[6],m[7],m[12],m[14],m[15])/det;
    inv.m[10]= det33(m[0],m[1],m[3],m[4],m[5],m[7],m[12],m[13],m[15])/det;
    inv.m[14]=-det33(m[0],m[1],m[2],m[4],m[5],m[6],m[12],m[13],m[14])/det;
    inv.m[3]=-det33(m[1],m[2],m[3],m[5],m[6],m[7],m[9],m[10],m[11])/det;
    inv.m[7]= det33(m[0],m[2],m[3],m[4],m[6],m[7],m[8],m[10],m[11])/det;
    inv.m[11]=-det33(m[0],m[1],m[3],m[4],m[5],m[7],m[8],m[9], m[11])/det;
    inv.m[15]= det33(m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9], m[10])/det;
}

// =============================================================================
// SECTION 2 — CATOM3D GEOMETRY CONSTANTS
// =============================================================================

static const double CONNECTOR_POS[12][3] = {
    { 1,    0,    0      }, { 0,    1,    0      },
    { 0.5,  0.5,  M_SQRT2/2.0 }, {-0.5,  0.5,  M_SQRT2/2.0 },
    {-0.5, -0.5,  M_SQRT2/2.0 }, { 0.5, -0.5,  M_SQRT2/2.0 },
    {-1,    0,    0      }, { 0,   -1,    0      },
    {-0.5, -0.5, -M_SQRT2/2.0 }, { 0.5, -0.5, -M_SQRT2/2.0 },
    { 0.5,  0.5, -M_SQRT2/2.0 }, {-0.5,  0.5, -M_SQRT2/2.0 }
};

static const double ORIENTATION_ANGLES[12][3] = {
    {  0,   0,   0}, {180,  0, -90}, {-90, 45, -45}, { 90, 45,-135},
    {-90,  45, 135}, { 90, 45,  45}, {  0,  0, 180}, {180,  0,  90},
    { 90, -45, 135}, {-90,-45,  45}, { 90,-45, -45}, {-90,-45,-135}
};

static const int TAB_CON3[8][3] = {
    {0,2,5},{0,9,10},{1,2,3},{1,10,11},{3,4,6},{6,8,11},{4,5,7},{7,8,9}
};
static const int TAB_CON4[6][4] = {
    {0,1,2,10},{1,3,6,11},{4,6,7,8},{0,5,7,9},{8,9,10,11},{2,3,4,5}
};

// =============================================================================
// SECTION 3 — FCC LATTICE HELPERS
// =============================================================================

static const double LATTICE_SCALE = 0.10;
static const int ZLINE = 2;

static const int FCC_EVEN[12][3] = {
    { 1, 0, 0}, { 0, 1, 0}, { 0, 0, 1}, {-1, 0, 1},
    {-1,-1, 1}, { 0,-1, 1}, {-1, 0, 0}, { 0,-1, 0},
    {-1,-1,-1}, { 0,-1,-1}, { 0, 0,-1}, {-1, 0,-1}
};
static const int FCC_ODD[12][3] = {
    { 1, 0, 0}, { 0, 1, 0}, { 1, 1, 1}, { 0, 1, 1},
    { 0, 0, 1}, { 1, 0, 1}, {-1, 0, 0}, { 0,-1, 0},
    { 0, 0,-1}, { 1, 0,-1}, { 1, 1,-1}, { 0, 1,-1}
};

struct GridPos { int x,y,z;
    bool operator==(const GridPos& o) const { return x==o.x&&y==o.y&&z==o.z; }
    bool operator!=(const GridPos& o) const { return !(*this==o); }
};

static void gridToWorld(GridPos g, double out[3]) {
    out[2] = (M_SQRT2/2.0) * (g.z + 0.5) * LATTICE_SCALE;
    if (g.z % 2 == 0) {
        out[0] = (g.x + 0.5) * LATTICE_SCALE;
        out[1] = (g.y + 0.5) * LATTICE_SCALE;
    } else {
        out[0] = (g.x + 1.0) * LATTICE_SCALE;
        out[1] = (g.y + 1.0) * LATTICE_SCALE;
    }
}

static GridPos worldToGrid(const double w[3]) {
    static const double R = 0.05;
    double scale = LATTICE_SCALE;
    int gz = (int)std::floor(w[2] / ((M_SQRT2/2.0) * scale) - 0.5 + R);
    int gx, gy;
    if (gz % 2 == 0) {
        double vx = w[0]/scale - 0.5; gx = (vx<0)?int(vx-R):int(vx+R);
        double vy = w[1]/scale - 0.5; gy = (vy<0)?int(vy-R):int(vy+R);
    } else {
        double vx = w[0]/scale - 1.0; gx = (vx<0)?int(vx-R):int(vx+R);
        double vy = w[1]/scale - 1.0; gy = (vy<0)?int(vy-R):int(vy+R);
    }
    return {gx,gy,gz};
}

static GridPos fccNeighbor(GridPos g, int con) {
    const int* d = (g.z % 2 == 0) ? FCC_EVEN[con] : FCC_ODD[con];
    return {g.x+d[0], g.y+d[1], g.z+d[2]};
}

static Mat4 orientationMatrix(int code) {
    int ori = code % 12;
    Mat4 Mz,My,Mx,M;
    Mz.setRotationZ(ORIENTATION_ANGLES[ori][2]);
    My.setRotationY(ORIENTATION_ANGLES[ori][1]);
    Mx.setRotationX(ORIENTATION_ANGLES[ori][0]);
    M = My * Mz;
    return Mx * M;
}
static Mat4 buildCatomMat(const double pos[3], int oriCode) {
    Mat4 T,R;
    T.setTranslation(pos[0],pos[1],pos[2]);
    R = orientationMatrix(oriCode);
    return T * R;
}

static int connectorForNeighbor(const Mat4& myMat, const double nPos[3]) {
    for(int i=0;i<12;i++){
        Vec3 local(CONNECTOR_POS[i][0]*LATTICE_SCALE,
                   CONNECTOR_POS[i][1]*LATTICE_SCALE,
                   CONNECTOR_POS[i][2]*LATTICE_SCALE, 1.0);
        Vec3 world = myMat * local;
        double dx=world[0]-nPos[0],dy=world[1]-nPos[1],dz=world[2]-nPos[2];
        if(dx*dx+dy*dy+dz*dz < 0.0015) return i;
    }
    return -1;
}

// =============================================================================
// SECTION 4 — MOTION RULE ENGINE
// =============================================================================

enum FaceType { HexaFace, OctaFace };

struct MotionLink {
    int from, to;
    FaceType faceType;
    double angle, radius;
    Vec3 axis1, axis2;
    std::vector<int> blocking;
};

static std::vector<MotionLink> g_links;

static const int* findTab3(int id1,int id2){
    for(int i=0;i<8;i++){ bool f1=false,f2=false;
        for(int j=0;j<3;j++){if(TAB_CON3[i][j]==id1)f1=true; if(TAB_CON3[i][j]==id2)f2=true;}
        if(f1&&f2)return TAB_CON3[i]; }
    assert(false); return nullptr;
}
static const int* findTab4(int id1,int id2){
    for(int i=0;i<6;i++){ bool f1=false,f2=false;
        for(int j=0;j<4;j++){if(TAB_CON4[i][j]==id1)f1=true; if(TAB_CON4[i][j]==id2)f2=true;}
        if(f1&&f2)return TAB_CON4[i]; }
    assert(false); return nullptr;
}

static void addLink(FaceType ft,int id1,int id2,double angle,double radius,
                    Vec3 ax1,Vec3 ax2,int nBC,const int* tabBC)
{
    Mat4 M=orientationMatrix(id1); Mat4 M1; M.inverse(M1);
    ax1=(M1*ax1).normalized(); ax2=(M1*ax2).normalized();
    MotionLink lnk; lnk.from=id1;lnk.to=id2;lnk.faceType=ft;
    lnk.angle=angle;lnk.radius=radius;lnk.axis1=ax1;lnk.axis2=ax2;
    for(int i=0;i<nBC;i++) if(tabBC[i]!=id2) lnk.blocking.push_back(tabBC[i]);
    g_links.push_back(lnk);
}

static void addLinks3(int id1,int id2,int id3,Vec3 ax1,Vec3 ax2,Vec3 ax3){
    static const double R=0.4530052159, A=std::atan(std::sqrt(2.)/2.)*180./M_PI;
    int b1[5],b2[5],b3[5]; const int* p;
    p=findTab4(id2,id3); for(int i=0;i<4;i++)b1[i]=p[i]; b1[4]=(id1+6)%12;
    p=findTab4(id1,id3); for(int i=0;i<4;i++)b2[i]=p[i]; b2[4]=(id2+6)%12;
    p=findTab4(id1,id2); for(int i=0;i<4;i++)b3[i]=p[i]; b3[4]=(id3+6)%12;
    addLink(HexaFace,id1,id2,A,R,-ax1,ax2,5,b1); addLink(HexaFace,id2,id1,A,R,-ax1,ax3,5,b2);
    addLink(HexaFace,id1,id3,A,R,-ax1,ax3,5,b1); addLink(HexaFace,id3,id1,A,R,-ax1,ax2,5,b3);
    addLink(HexaFace,id2,id3,A,R,-ax1,ax2,5,b2); addLink(HexaFace,id3,id2,A,R,-ax1,ax3,5,b3);
}
static void addLinks4(int id1,int id2,int id3,int id4,Vec3 left,Vec3 lup,Vec3 rup){
    static const double R=0.453081839321973, A=45.0;
    int b1[6],b2[6],b3[6],b4[6]; const int*p3; int j;
    auto fill=[&](int*bc,int a,int b,int c,int ex,int opp){
        p3=findTab3(a,b); for(int i=0;i<3;i++)bc[i]=p3[i];
        p3=findTab3(b,c); j=3; for(int i=0;i<3;i++)if(p3[i]!=ex)bc[j++]=p3[i]; bc[5]=opp;
    };
    fill(b1,id2,id3,id4,id3,(id1+6)%12); fill(b2,id3,id4,id1,id4,(id2+6)%12);
    fill(b3,id4,id1,id2,id1,(id3+6)%12); fill(b4,id1,id2,id3,id2,(id4+6)%12);
    addLink(OctaFace,id1,id2,A,R,-left, rup, 6,b1); addLink(OctaFace,id2,id1,A,R, left, lup, 6,b2);
    addLink(OctaFace,id1,id3,A,R,-left,-left,6,b1); addLink(OctaFace,id3,id1,A,R,-left,-left,6,b3);
    addLink(OctaFace,id1,id4,A,R,-left,-rup, 6,b1); addLink(OctaFace,id4,id1,A,R, left,-lup, 6,b4);
    addLink(OctaFace,id2,id3,A,R, left,-lup, 6,b2); addLink(OctaFace,id3,id2,A,R,-left,-rup,6,b3);
    addLink(OctaFace,id2,id4,A,R, left, left,6,b2); addLink(OctaFace,id4,id2,A,R, left, left,6,b4);
    addLink(OctaFace,id3,id4,A,R,-left, rup, 6,b3); addLink(OctaFace,id4,id3,A,R, left, lup, 6,b4);
}

static void buildMotionLinks() {
    Vec3 up(0,1,0),down(0,-1,0),
         upleft(1,1,M_SQRT2),upright(-1,1,-M_SQRT2),
         downright(1,-1,-M_SQRT2),downleft(-1,-1,M_SQRT2),
         left(0,0,1),right(0,0,-1),lup(-1,1,0),rup(1,1,0);
    addLinks3(1,2,3,up,downright,downleft); addLinks3(7,4,5,up,downright,downleft);
    addLinks3(0,5,2,down,upleft,upright);   addLinks3(6,3,4,down,upleft,upright);
    addLinks3(1,11,10,down,upleft,upright); addLinks3(7,9,8,down,upleft,upright);
    addLinks3(0,10,9,up,downright,downleft);addLinks3(6,8,11,up,downright,downleft);
    addLinks4(0,2,1,10,left,lup,rup);  addLinks4(6,4,7,8,left,lup,rup);
    addLinks4(1,3,6,11,right,-rup,-lup);addLinks4(7,5,0,9,right,-rup,-lup);
    addLinks4(2,5,4,3,left,lup,rup);   addLinks4(8,9,10,11,left,lup,rup);
}

// =============================================================================
// SECTION 5 — ROTATION ANIMATOR (port of Catoms3DRotation)
// =============================================================================

static const int N_STEPS = 20;

struct RotationAnim {
    Vec3 axe1,axe2; double angle1,angle2;
    Vec3 A0C0,A0D0,A1C1,A1D1;
    Mat4 initialMat, finalMat;
    int step; bool firstRotation;
};

static RotationAnim buildRotationAnim(const Mat4& myMat, const Mat4& pivotMat,
                                       const MotionLink& lnk)
{
    RotationAnim r;
    r.angle1=lnk.angle; r.angle2=lnk.angle;
    r.step=0; r.firstRotation=true; r.initialMat=myMat;
    static const double c_2=1.0/(3.0+std::sqrt(2.0));

    Mat4 MA=myMat,MA1; MA.inverse(MA1);
    Vec3 AB=MA1*pivotMat.getPosition();
    Vec3 ABv(AB[0],AB[1],AB[2],0.0);
    r.axe1=lnk.axis1.normalized();
    double rlen=ABv.norm()/2.0;
    double shift=(r.angle1>0)?c_2*rlen:-c_2*rlen;
    Vec3 V=ABv.cross(r.axe1).normalized();
    r.A0D0=(0.5+0.5*lnk.radius)*ABv+shift*V;
    r.A0C0=(0.5-0.5*lnk.radius)*ABv+shift*V;

    Mat4 mr; mr.setRotation(r.angle1,r.axe1);
    Mat4 mTAB; mTAB.setTranslation(ABv);
    Mat4 mTBA; mTBA.setTranslation(-ABv[0],-ABv[1],-ABv[2]);
    Mat4 fLoc1=mTAB*(mr*(mTBA*mr));

    Mat4 MA2=MA*fLoc1, MA2i; MA2.inverse(MA2i);
    Vec3 AB2=MA2i*pivotMat.getPosition();
    Vec3 AB2v(AB2[0],AB2[1],AB2[2],0.0);

    Mat4 fInv; fLoc1.inverse(fInv);
    r.axe2=(fInv*lnk.axis2).normalized();
    Mat4 mr2; mr2.setRotation(r.angle2,r.axe2);
    Mat4 mTAB2; mTAB2.setTranslation(AB2v);
    Mat4 mTBA2; mTBA2.setTranslation(-AB2v[0],-AB2v[1],-AB2v[2]);
    Mat4 fLoc2=mTAB2*(mr2*(mTBA2*mr2));
    Mat4 fLoc=fLoc1*fLoc2;
    r.finalMat=MA*fLoc;

    Mat4 MA3=MA*fLoc, MA3i; MA3.inverse(MA3i);
    Vec3 AB3=MA3i*pivotMat.getPosition();
    Vec3 AB3v(AB3[0],AB3[1],AB3[2],0.0);
    double shift2=(r.angle2>0)?-c_2*rlen:c_2*rlen;
    Vec3 V2=AB3v.cross(r.axe2).normalized();
    r.A1D1=(0.5+0.5*lnk.radius)*AB3v+shift2*V2;
    r.A1C1=(0.5-0.5*lnk.radius)*AB3v+shift2*V2;
    return r;
}

static bool rotationNextStep(RotationAnim& r, Mat4& out) {
    if(r.firstRotation){
        r.step++;
        double angle=r.angle1*r.step/N_STEPS;
        Mat4 mr; mr.setRotation(angle,r.axe1);
        Mat4 tCA,tDC,tAD;
        tCA.setTranslation(-r.A0C0[0],-r.A0C0[1],-r.A0C0[2]);
        tDC.setTranslation(r.A0C0[0]-r.A0D0[0],r.A0C0[1]-r.A0D0[1],r.A0C0[2]-r.A0D0[2]);
        tAD.setTranslation(r.A0D0[0],r.A0D0[1],r.A0D0[2]);
        out=r.initialMat*(tAD*(mr*(tDC*(mr*tCA))));
        if(r.step==N_STEPS) r.firstRotation=false;
    } else {
        r.step--;
        double angle=-r.angle2*r.step/N_STEPS;
        Mat4 mr; mr.setRotation(angle,r.axe2);
        Mat4 tCA,tDC,tAD;
        tCA.setTranslation(-r.A1C1[0],-r.A1C1[1],-r.A1C1[2]);
        tDC.setTranslation(r.A1C1[0]-r.A1D1[0],r.A1C1[1]-r.A1D1[1],r.A1C1[2]-r.A1D1[2]);
        tAD.setTranslation(r.A1D1[0],r.A1D1[1],r.A1D1[2]);
        out=r.finalMat*(tAD*(mr*(tDC*(mr*tCA))));
    }
    return r.step==0;
}

static void matToPos(const Mat4& M, double p[3]){ p[0]=M.m[3];p[1]=M.m[7];p[2]=M.m[11]; }
static void matToAxisAngle(const Mat4& M, double aa[4]){
    double tr=M.m[0]+M.m[5]+M.m[10];
    double angle=std::acos(std::max(-1.0,std::min(1.0,(tr-1.0)*0.5)));
    if(std::abs(angle)<1e-9){aa[0]=0;aa[1]=1;aa[2]=0;aa[3]=0;return;}
    aa[0]=(M.m[9]-M.m[6])/(2.*std::sin(angle));
    aa[1]=(M.m[2]-M.m[8])/(2.*std::sin(angle));
    aa[2]=(M.m[4]-M.m[1])/(2.*std::sin(angle));
    aa[3]=angle;
    double n=std::sqrt(aa[0]*aa[0]+aa[1]*aa[1]+aa[2]*aa[2]);
    if(n>1e-12){aa[0]/=n;aa[1]/=n;aa[2]/=n;}
}

static int oriFromMat(const Mat4& M) {
    double psmax=-1; int best=0;
    for(int i=0;i<12;i++){
        Vec3 local(CONNECTOR_POS[i][0],CONNECTOR_POS[i][1],CONNECTOR_POS[i][2],0.0);
        Vec3 world=M*local;
        if(world[0]>psmax){psmax=world[0];best=i;}
    }
    return best;
}

// =============================================================================
// SECTION 6 — TRACE PLAYER HELPERS
// =============================================================================

struct Move {
    long long t_us;
    GridPos init, from, to, pivot;
};

// Load trace.csv, keeping only the moves whose initial cell is myInit, sorted by time.
// If myInit is null, keep all moves.
static std::vector<Move> loadTraceFrom(const char* path, const GridPos* myInit) {
    std::vector<Move> out;
    std::ifstream f(path);
    if (!f.is_open()) return out;

    std::string line;
    std::getline(f, line); // skip header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        long long v[14]; int k = 0;
        while (k < 14 && std::getline(ss, tok, ',')) {
            try { v[k++] = std::stoll(tok); } catch (...) { k = 0; break; }
        }
        if (k < 14) continue;
        Move m;
        m.t_us  = v[0];
        m.init  = {(int)v[2],  (int)v[3],  (int)v[4]};
        m.from  = {(int)v[5],  (int)v[6],  (int)v[7]};
        m.to    = {(int)v[8],  (int)v[9],  (int)v[10]};
        m.pivot = {(int)v[11], (int)v[12], (int)v[13]};
        if (!myInit || m.init == *myInit) out.push_back(m);
    }
    std::sort(out.begin(), out.end(),
              [](const Move& a, const Move& b){ return a.t_us < b.t_us; });
    return out;
}

// Find the rotation link that takes myMat onto toCell, rolling around pivotCell.
// buildRotationAnim uses only the pivot POSITION, so the pivot orientation is
// irrelevant.  Returns false if no link lands on the target.
static bool findAnimForMove(const Mat4& myMat, GridPos pivotCell, GridPos toCell,
                            RotationAnim& out)
{
    double pivW[3]; gridToWorld(pivotCell, pivW);
    double tgtW[3]; gridToWorld(toCell, tgtW);

    int con = connectorForNeighbor(myMat, pivW);
    if (con < 0) return false;

    Mat4 pivMat; pivMat.setTranslation(pivW[0], pivW[1], pivW[2]);

    double best = std::numeric_limits<double>::max();
    bool found = false;
    RotationAnim bestAnim;
    for (const MotionLink& lnk : g_links) {
        if (lnk.from != con) continue;
        RotationAnim anim = buildRotationAnim(myMat, pivMat, lnk);
        Vec3 fp = anim.finalMat.getPosition();
        double dx=fp[0]-tgtW[0], dy=fp[1]-tgtW[1], dz=fp[2]-tgtW[2];
        double d = dx*dx+dy*dy+dz*dz;
        if (d < best) { best = d; bestAnim = anim; found = true; }
    }
    if (found && best < 0.002) { out = bestAnim; return true; }
    return false;
}

// =============================================================================
// SECTION 7 — STRUCTURAL INTEGRITY ANALYSIS
// Pure geometry/graph (no Webots): given the set of occupied FCC cells, report
// structural-integrity indicators. Used by the Webots analyst and the offline
// verifier alike.
//   * connected components  (a split structure has fallen apart)
//   * articulation points   (modules whose loss disconnects the structure = SPOFs)
//   * unsupported/overhang   (no module or floor beneath them)
//   * centre-of-mass over the support footprint (tip-over check)
// =============================================================================

struct StructuralReport {
    int modules=0, components=0, articulation=0, supported=0, overhanging=0;
    double comX=0, comY=0; bool comInSupport=true;
};

// A connector is "down-pointing" if its FCC offset has dz < 0 (used for support).
static bool isDownConnector(GridPos g, int con) {
    const int* d = (g.z % 2 == 0) ? FCC_EVEN[con] : FCC_ODD[con];
    return d[2] < 0;
}

static StructuralReport analyzeStructure(const std::vector<GridPos>& cells) {
    StructuralReport r;
    r.modules = (int)cells.size();
    if (cells.empty()) return r;

    std::map<std::tuple<int,int,int>,int> idx;
    for (int i=0;i<(int)cells.size();++i)
        idx[std::make_tuple(cells[i].x,cells[i].y,cells[i].z)] = i;
    auto find=[&](GridPos g)->int{
        auto it=idx.find(std::make_tuple(g.x,g.y,g.z));
        return it==idx.end()?-1:it->second;
    };

    std::vector<std::vector<int>> adj(cells.size());
    for (int i=0;i<(int)cells.size();++i)
        for (int c=0;c<12;++c){
            int j=find(fccNeighbor(cells[i],c));
            if(j>=0) adj[i].push_back(j);
        }

    // connected components
    std::vector<int> comp(cells.size(),-1); int nc=0;
    for(int i=0;i<(int)cells.size();++i){
        if(comp[i]!=-1) continue;
        nc++; std::vector<int> st{i}; comp[i]=nc;
        while(!st.empty()){int u=st.back();st.pop_back();
            for(int v:adj[u]) if(comp[v]==-1){comp[v]=nc;st.push_back(v);}}
    }
    r.components=nc;

    // articulation points (Tarjan)
    std::vector<int> disc(cells.size(),0), low(cells.size(),0);
    std::vector<bool> ap(cells.size(),false); int timer=0;
    std::function<void(int,int)> dfs=[&](int u,int parent){
        disc[u]=low[u]=++timer; int children=0;
        for(int v:adj[u]){
            if(!disc[v]){children++; dfs(v,u);
                low[u]=std::min(low[u],low[v]);
                if(parent!=-1 && low[v]>=disc[u]) ap[u]=true;
            } else if(v!=parent) low[u]=std::min(low[u],disc[v]);
        }
        if(parent==-1 && children>1) ap[u]=true;
    };
    for(int i=0;i<(int)cells.size();++i) if(!disc[i]) dfs(i,-1);
    for(int i=0;i<(int)cells.size();++i) if(ap[i]) r.articulation++;

    // support / overhang + centre of mass over base footprint
    int minz=cells[0].z; for(auto&c:cells) minz=std::min(minz,c.z);
    double sx=0,sy=0, bxmin=1e9,bxmax=-1e9,bymin=1e9,bymax=-1e9;
    for(int i=0;i<(int)cells.size();++i){
        double w[3]; gridToWorld(cells[i],w); sx+=w[0]; sy+=w[1];
        bool sup = (cells[i].z==minz);
        if(!sup) for(int c=0;c<12;++c)
            if(isDownConnector(cells[i],c) && find(fccNeighbor(cells[i],c))>=0){sup=true;break;}
        if(sup) r.supported++; else r.overhanging++;
        if(cells[i].z==minz){bxmin=std::min(bxmin,w[0]);bxmax=std::max(bxmax,w[0]);
                             bymin=std::min(bymin,w[1]);bymax=std::max(bymax,w[1]);}
    }
    r.comX=sx/cells.size(); r.comY=sy/cells.size();
    const double margin=LATTICE_SCALE*0.5;
    r.comInSupport = (r.comX>=bxmin-margin && r.comX<=bxmax+margin &&
                      r.comY>=bymin-margin && r.comY<=bymax+margin);
    return r;
}

#endif // CATOM3D_CORE_HPP_
