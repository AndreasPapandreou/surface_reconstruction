#include "canvas.h"
#include "mesh.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <QtOpenGL>
#include <MathGeoLib.h>

using namespace std;
using namespace vvr;

/////////////////////////////////////////////////////////////////////////////////////////
//! Global constants
/////////////////////////////////////////////////////////////////////////////////////////

float Shape::DEF_LINE_WIDTH = 2.2f;
float Shape::DEF_POINT_SIZE = 7.0f;

/////////////////////////////////////////////////////////////////////////////////////////
//! Common Color Definitions
/////////////////////////////////////////////////////////////////////////////////////////

const Colour Colour::red            (0xFF, 0x00, 0x00);
const Colour Colour::blue           (0x00, 0x00, 0xFF);
const Colour Colour::grey           (0x66, 0x66, 0x66);
const Colour Colour::cyan           (0x00, 0xFF, 0xFF);
const Colour Colour::white          (0xFF, 0xFF, 0xFF);
const Colour Colour::green          (0x00, 0xFF, 0x00);
const Colour Colour::black          (0x00, 0x00, 0x00);
const Colour Colour::yellow         (0xFF, 0xFF, 0x00);
const Colour Colour::orange         (0xFF, 0x66, 0x00);
const Colour Colour::magenta        (0xFF, 0x00, 0xFF);
const Colour Colour::darkRed        (0x8B, 0x00, 0x00);
const Colour Colour::darkOrange     (0xFF, 0x8C, 0x00);
const Colour Colour::darkGreen      (0x00, 0x64, 0x00);
const Colour Colour::yellowGreen    (0x9A, 0xCD, 0x32);

/////////////////////////////////////////////////////////////////////////////////////////
//! vvr::Shape and childs
/////////////////////////////////////////////////////////////////////////////////////////

void Shape::draw() const 
{
    glPolygonMode(GL_FRONT_AND_BACK, b_render_solid ? GL_FILL : GL_LINE);
    glColor3ubv(colour.data);
    drawShape();
}

void Point2D::drawShape() const 
{
    glPointSize(DEF_POINT_SIZE);
    glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
    glVertex2f(x,y);
    glEnd();
}

void Point3D::drawShape() const 
{
    glPointSize(DEF_POINT_SIZE);
    glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
    glVertex3f(x,y,z);
    glEnd();
}

void LineSeg2D::drawShape() const
{
    glLineWidth(DEF_LINE_WIDTH);
    glBegin(GL_LINES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glEnd();
}

void Line2D::drawShape() const 
{
    double dx = x2-x1;
    double dy = y2-y1;

    glLineWidth(DEF_LINE_WIDTH);
    glBegin(GL_LINES);
    glVertex2f(x1 - 1000*dx, y1 - 1000*dy);
    glVertex2f(x2 + 1000*dx, y2 + 1000*dy);
    glEnd();
}

void LineSeg3D::drawShape() const 
{
    double dx = x2-x1;
    double dy = y2-y1;

    glLineWidth(DEF_LINE_WIDTH);
    glBegin(GL_LINES);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glEnd();
}

void Circle2D::drawShape() const 
{
    if (rad_from >= rad_to) {
        std::cerr << "Trying to render circle with [rad_from >= rad_to]" << std::endl;
        return;
    }

    double x_, y_;
    unsigned const numOfSegments = 60;

    glLineWidth(DEF_LINE_WIDTH);
    glBegin(b_render_solid? GL_POLYGON : (closed_loop?GL_LINE_LOOP:GL_LINE_STRIP));
    double d_th = (rad_to - rad_from) / numOfSegments;
    for(double theta = rad_from; theta <= rad_to; theta+=d_th) {
        x_ = r * cosf(theta);
        y_ = r * sinf(theta);
        glVertex2f(x + x_, y + y_);
    }
    glEnd();

}

void Sphere3D::drawShape() const 
{
    glPushMatrix();
    glTranslated(x, y, z);
    glScaled(rad, rad, rad);
    drawSphere(rad, 12, 15);
    glPopMatrix();
}

Box3D::Box3D(const std::vector<vec> vertices, const Colour &col)
    : Shape(col)
    , transparency(0)
{
    math::AABB aabb = aabbFromVertices(vertices);

    x1 = aabb.minPoint.x;
    y1 = aabb.minPoint.y;
    z1 = aabb.minPoint.z;

    x2 = aabb.maxPoint.x;
    y2 = aabb.maxPoint.y;
    z2 = aabb.maxPoint.z;
}

void Box3D::drawShape() const 
{
    drawBox(x1, y1, z1, x2, y2, z2, colour, 0);
    drawBox(x1, y1, z1, x2, y2, z2, colour, 255 - transparency * 255);
}

void Triangle2D::drawShape() const 
{
    glLineWidth(DEF_LINE_WIDTH);
    glBegin(GL_TRIANGLES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glVertex2f(x3, y3);
    glEnd();
}

void Triangle3D::drawShape() const 
{
    glLineWidth(DEF_LINE_WIDTH);
    glBegin(GL_TRIANGLES);
    math::vec n = math::Triangle(
        math::vec(x1,y1,z1),
        math::vec(x2,y2,z2),
        math::vec(x3,y3,z3)
    ).NormalCW();
    
    glNormal3fv(n.ptr());

    glColor3ubv(vertex_col[0].data);
    glVertex3f(x1, y1, z1);
    
    glColor3ubv(vertex_col[1].data);
    glVertex3f(x2, y2, z2);

    glColor3ubv(vertex_col[2].data);
    glVertex3f(x3, y3, z3);

    glEnd();
}

/////////////////////////////////////////////////////////////////////////////////////////
//! vvr::Frame vvr::Canvas
/////////////////////////////////////////////////////////////////////////////////////////

Frame::Frame() : show_old(true)
{

}

Frame::Frame(bool show_old) : show_old(show_old)
{

}

Canvas2D::Canvas2D()
{
    clear();
}

Canvas2D::~Canvas2D()
{
    for (int fi=0; fi<frames.size(); fi++) {
        for (int si=0; si<frames[fi].shapes.size(); si++) {
            delete frames[fi].shapes[si];
        }
    }
}

void Canvas2D::add(Shape *shape_ptr) 
{
    frames[fi].shapes.push_back(shape_ptr);
}

void Canvas2D::newFrame(bool show_old_frames) 
{
    frames.push_back(Frame(show_old_frames));
    ff();
}

void Canvas2D::draw() 
{
    Frame *frame;
    int fi_ = (int) fi;
    while (frames[fi_].show_old && --fi_>=0);
    while(fi_ <= fi) {
        frame = &frames[fi_];
        for (unsigned i=0; i<frames[fi_].shapes.size(); i++)
            frames[fi_].shapes[i]->draw();
        fi_++;
    }
}

void Canvas2D::next() 
{
    if (fi<frames.size()-1) fi++;
}

void Canvas2D::prev() 
{
    if (fi>0) fi--;
}

void Canvas2D::rew() 
{
    fi = 0;
}

void Canvas2D::ff() 
{
    fi = frames.size()-1;
}

void Canvas2D::resize(int i) 
{
    if (i<1 || i > size()-1) return;

    // Delete shapes of frames that will be discarded
    for (int fi=i; fi<frames.size(); fi++) {
        for (int si=0; si<frames[fi].shapes.size(); si++) {
            delete frames[fi].shapes[si];
        }
    }

    frames.resize(i);
    fi=i-1;
}

void Canvas2D::clear()
{
    // Delete shapes of frames that will be discarded
    for (int fi=0; fi<frames.size(); fi++) {
        for (int si=0; si<frames[fi].shapes.size(); si++) {
            delete frames[fi].shapes[si];
        }
    }

    frames.clear();
    frames.push_back(Frame(false));
    fi=0;
}

void Canvas2D::clearFrame()
{
    for (int si = 0; si<frames[fi].shapes.size(); si++) {
        delete frames[fi].shapes[si];
    }

    frames[fi].shapes.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////
//! Drawing Utilities for 3rd party lib objects
/////////////////////////////////////////////////////////////////////////////////////////

void vvr::draw(C2DPointSet &point_set, const Colour &col)
{
    /* Draw point cloud */
    for (int i = 0; i < point_set.size(); i++) {
        Point2D(
            point_set.GetAt(i)->x,
            point_set.GetAt(i)->y,
            col).draw();
    }
}

void vvr::draw(C2DLineSet  &line_set, const Colour &col)
{
    for (int i = 0; i < line_set.size(); i++) {
        LineSeg2D(
            line_set.GetAt(i)->GetPointFrom().x,
            line_set.GetAt(i)->GetPointFrom().y,
            line_set.GetAt(i)->GetPointTo().x,
            line_set.GetAt(i)->GetPointTo().y,
            col).draw();
    }
}

void vvr::draw(C2DPolygon  &polygon, const Colour &col, bool filled)
{
    bool err = false;

    if (filled)
    {
        if (!polygon.IsConvex()) {
            err = !polygon.CreateConvexSubAreas();
        }
        if (!err) 
        {
            C2DPolygonSet polyset;
            polygon.GetConvexSubAreas(polyset);
            for (int i = 0; i < polyset.size(); i++) {
                C2DPolygon &convpoly = *polyset.GetAt(i);
                C2DPoint convpoly_centroid = convpoly.GetCentroid();
                for (int j = 0; j < convpoly.GetLines().size(); j++) {
                    Triangle2D t(
                        convpoly.GetLines().GetAt(j)->GetPointFrom().x,
                        convpoly.GetLines().GetAt(j)->GetPointFrom().y,
                        convpoly.GetLines().GetAt(j)->GetPointTo().x,
                        convpoly.GetLines().GetAt(j)->GetPointTo().y,
                        convpoly_centroid.x, convpoly_centroid.y);
                    t.setSolidRender(true);
                    t.setColour(col);
                    t.draw();
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < polygon.GetLines().size(); i++) {
            LineSeg2D(
                polygon.GetLines().GetAt(i)->GetPointFrom().x,
                polygon.GetLines().GetAt(i)->GetPointFrom().y,
                polygon.GetLines().GetAt(i)->GetPointTo().x,
                polygon.GetLines().GetAt(i)->GetPointTo().y,
                col).draw();
        }
    }

    if (err) {
        std::cerr << "Polygon Invalid. Cannot render." << std::endl;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//! Geometric struct converters from 3rd party lib objects
/////////////////////////////////////////////////////////////////////////////////////////

vvr::Triangle3D vvr::math2vvr(const math::Triangle &t, const vvr::Colour &col)
{
    return vvr::Triangle3D(
        t.a.x, t.a.y, t.a.z,
        t.c.x, t.c.y, t.c.z,
        t.b.x, t.b.y, t.b.z,
        col);
}

vvr::LineSeg3D vvr::math2vvr(const math::LineSegment &l, const vvr::Colour &col)
{
    return vvr::LineSeg3D(
        l.a.x, l.a.y, l.a.z,
        l.b.x, l.b.y, l.b.z,
        col);
}

vvr::LineSeg3D vvr::math2vvr(const math::Line &l, const vvr::Colour &col)
{
    const auto &lseg(l.ToLineSegment(1000));
    return math2vvr(lseg, col);
}

vvr::Point3D vvr::math2vvr(const math::vec &v, const vvr::Colour &col)
{
    return vvr::Point3D(v.x, v.y, v.z, col);
}

/////////////////////////////////////////////////////////////////////////////////////////
//! Private drawing utils
/////////////////////////////////////////////////////////////////////////////////////////

void vvr::drawSphere(double r, int lats, int longs)
{
    int i, j;
    for (i = 0; i <= lats; i++) {
        double lat0 = M_PI * (-0.5 + (double)(i - 1) / lats);
        double z0 = sin(lat0);
        double zr0 = cos(lat0);

        double lat1 = M_PI * (-0.5 + (double)i / lats);
        double z1 = sin(lat1);
        double zr1 = cos(lat1);
        glBegin(GL_QUAD_STRIP);
        for (j = 0; j <= longs; j++) {
            double lng = 2 * M_PI * (double)(j - 1) / longs;
            double x = cos(lng);
            double y = sin(lng);
            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(x * zr0, y * zr0, z0);
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(x * zr1, y * zr1, z1);
        }
        glEnd();
    }
}

void vvr::drawBox(double x1, double y1, double z1, double x2, double y2, double z2, Colour col, char a)
{
    static vec p[8];
    vec *v = p;

    *v++ = vec(x1, y1, z1); //0
    *v++ = vec(x1, y2, z1); //1
    *v++ = vec(x1, y2, z2); //2
    *v++ = vec(x1, y1, z2); //3
    *v++ = vec(x2, y1, z1); //4
    *v++ = vec(x2, y2, z1); //5
    *v++ = vec(x2, y2, z2); //6
    *v++ = vec(x2, y1, z2); //7

    glPolygonMode(GL_FRONT_AND_BACK, a ? GL_FILL : GL_LINE);
    glBegin(GL_QUADS);

    if (a) {
        glColor4ub(col.r, col.g, col.b, a);
    }
    else {
        glColor3ubv(col.data);
    }

    glVertex3fv(p[0].ptr());
    glVertex3fv(p[1].ptr());
    glVertex3fv(p[2].ptr());
    glVertex3fv(p[3].ptr());

    glVertex3fv(p[1].ptr());
    glVertex3fv(p[2].ptr());
    glVertex3fv(p[6].ptr());
    glVertex3fv(p[5].ptr());

    glVertex3fv(p[4].ptr());
    glVertex3fv(p[5].ptr());
    glVertex3fv(p[6].ptr());
    glVertex3fv(p[7].ptr());

    glVertex3fv(p[0].ptr());
    glVertex3fv(p[4].ptr());
    glVertex3fv(p[7].ptr());
    glVertex3fv(p[3].ptr());

    glVertex3fv(p[2].ptr());
    glVertex3fv(p[3].ptr());
    glVertex3fv(p[7].ptr());
    glVertex3fv(p[6].ptr());

    glVertex3fv(p[0].ptr());
    glVertex3fv(p[1].ptr());
    glVertex3fv(p[5].ptr());
    glVertex3fv(p[4].ptr());

    glEnd();
}
