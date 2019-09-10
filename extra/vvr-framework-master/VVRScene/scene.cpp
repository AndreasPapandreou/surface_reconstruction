#include "scene.h"
#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <string>
#include <algorithm>
#include <locale>
#include <cmath>
#include <QtOpenGL>
#include <MathGeoLib.h>

using namespace vvr;
using namespace std;
using namespace math;

#define VVR_FOV_MAX 160
#define VVR_FOV_MIN 2

//! Scene::

Scene::Scene()
{
    m_perspective_proj = false;
    m_fullscreen = false;
    m_create_menus = true;
    m_hide_log = true;
    m_hide_sliders = true;
    m_camera_dist = 100;
    m_fov = 30;
    setCameraPos(vec(0, 0, m_camera_dist));
}

void Scene::reset()
{
    setCameraPos(vec(0, 0, m_camera_dist));
}

void Scene::setCameraPos(const vec &pos)
{
    vec front0 = vec(0, 0, -1);
    vec front = pos.Neg().Normalized();
    vec up(0, 1, 0);

    if (!front0.Equals(front)) {
        float3x3 cam_rot = float3x3::RotateFromTo(front0, front);
        up = cam_rot * up;
    }

    m_frustum.SetKind(FrustumSpaceGL, FrustumRightHanded);
    m_frustum.SetFrame(pos, front, up);
}

/////////////////////////////////////////////////////////////////////////////////////////
//! OpenGL Callbacks
/////////////////////////////////////////////////////////////////////////////////////////

static void GL_Info()
{
    /* PRINT OpenGL INFO */
    printf("\n=== VVR Framework ================\n");
    printf(" %s\n", glGetString(GL_VERSION));
    printf(" %s\n", glGetString(GL_VENDOR));
    printf(" %s\n", glGetString(GL_RENDERER));
    printf("==================================\n\n");
    fflush(0);
}

void Scene::GL_Init()
{
    // Light setup
    float lz = m_camera_dist * 3;
    static GLfloat light_position[] = { 0, 0, lz, 1 };
    static GLfloat ambientLight[] = { .75, .75, .75, 1 };
    static GLfloat diffuseLight[] = { .75, .75, .75, 1 };
    static GLfloat specularLight[] = { .85, .85, .85, 1 };

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);

    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LEQUAL);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glShadeModel(GL_SMOOTH);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    GL_Info();
}

void Scene::GL_Resize(int w, int h)
{
    const float ar = (float)w / h;
    m_screen_width = w;
    m_screen_height = h;

    if (m_perspective_proj)
    {
        const float n = m_camera_dist * 0.1;
        const float f = -m_camera_dist * 2.1;
        m_frustum.SetVerticalFovAndAspectRatio(DegToRad(m_fov), ar);
        m_frustum.SetViewPlaneDistances(n, f);
        m_scene_width = m_frustum.NearPlaneWidth() / 0.1;
        m_scene_height = m_frustum.NearPlaneHeight() / 0.1;
    }
    else
    {
        const float n = -m_camera_dist * 2;
        const float f = m_camera_dist * 2;
        m_scene_width = m_camera_dist / 2;
        m_scene_height = m_scene_width / ar;
        m_frustum.SetOrthographic(m_scene_width, m_scene_height);
        m_frustum.SetViewPlaneDistances(n, f);
    }

    //! Set OpenGL Projection Matrix
    glViewport(0, 0, m_screen_width, m_screen_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float4x4 pjm = m_frustum.ProjectionMatrix();
    pjm.Transpose(); // Covert to colunm major for OpenGL
    glMultMatrixf(pjm.ptr());
    resize();
}

void Scene::GL_Render()
{
    glClearColor(m_bg_col.r / 255.0, m_bg_col.g / 255.0, m_bg_col.b / 255.0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float4x4 mvm = m_frustum.ViewMatrix();
    mvm.Transpose(); // Covert to colunm major for OpenGL
    glMultMatrixf(mvm.ptr());
    draw();
}

void Scene::drawAxes()
{
    GLfloat len = 2.0 * getSceneWidth();

    glBegin(GL_LINES);

    // X
    glColor3ub(0xFF, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(len, 0, 0);

    // Y
    glColor3f(0, 0xFF, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, len, 0);

    // Z
    glColor3f(0, 0, 0xFF);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, len);

    glEnd();
}

void Scene::enterPixelMode()
{
    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-m_screen_width / 2, m_screen_width / 2, -m_screen_height / 2, m_screen_height / 2, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
}

void Scene::returnFromPixelMode()
{
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}

void Scene::mouse2pix(int &x, int &y)
{
    x -= m_screen_width / 2;
    y -= m_screen_height / 2;
    // Reverse the default window coordinate system so that y grows upwards.
    y = -y;
}

Ray Scene::unproject(int x, int y)
{
    return m_frustum.UnProject(
        (float)x / getViewportWidth() * 2,
        (float)y / getViewportHeight() * 2);
}

/////////////////////////////////////////////////////////////////////////////////////////
//! UI
/////////////////////////////////////////////////////////////////////////////////////////

void Scene::keyEvent(unsigned char key, bool up, int modif)
{
    if (up) return;

    int ctrl = modif & 0x01;
    int shift = modif & 0x02;

    switch (isprint(key) ? tolower(key) : key) {
    case 'r': this->reset(); break;
    case '2': setCameraPos(vec(0, -m_camera_dist, 0)); break;
    case '4': setCameraPos(vec(-m_camera_dist, 0, 0)); break;
    case '6': setCameraPos(vec(m_camera_dist, 0, 0)); break;
    case '8': setCameraPos(vec(0, m_camera_dist, 0)); break;
    case '5': setCameraPos(vec(0, 0, m_camera_dist)); break;
    }
}

void Scene::arrowEvent(ArrowDir dir, int modif)
{
    int ctrl = modif & 0x01;
    int shift = modif & 0x02;

}

void Scene::mousePressed(int x, int y, int modif)
{
    m_mouselastX = x;
    m_mouselastY = y;
}

void Scene::mouseMoved(int x, int y, int modif)
{
    const int dx = x - m_mouselastX;
    const int dy = y - m_mouselastY;
    m_mouselastX = x;
    m_mouselastY = y;

    const math::vec up = m_frustum.Up();
    const math::vec right = m_frustum.WorldRight();

    float angleYaw = math::DegToRad(-vvr::normalizeAngle((float)dx / 2));
    float anglePitch = math::DegToRad(vvr::normalizeAngle((float)dy / 2));

    math::float3x3 transform = float3x3::RotateAxisAngle(right, anglePitch) *
        float3x3::RotateAxisAngle(up, angleYaw);

    m_frustum.Transform(transform);
    math::vec newpos = m_frustum.Pos();
    math::vec newfront = m_frustum.Front();
    math::vec newup = m_frustum.Up();
    m_frustum.SetFrame(newpos, newfront, newup);
}

void Scene::mouseReleased(int x, int y, int modif)
{

}

void Scene::mouseWheel(int dir, int modif)
{
    m_fov += -4.0 * dir;
    if (m_fov < VVR_FOV_MIN) m_fov = VVR_FOV_MIN;
    else if (m_fov > VVR_FOV_MAX) m_fov = VVR_FOV_MAX;
    GL_Resize(m_screen_width, m_screen_height);
}

void Scene::sliderChanged(int slider_id, float val)
{

}

void Scene::setSliderVal(int slider_id, float val)
{
    if (slider_id > 5 || slider_id < 0) return;
    if (val > 1) val = 1;
    else if (val < 0) val = 0;
    //TODO: Connect scene object backwards to window in order to be able to change 
    //      slider values from the scene.
}

//! Ground::

Ground::Ground(const float W, const float D, const float B, const float T, const vvr::Colour &colour)
    : m_col(colour)
{
    const vec vA(-W / 2, B, -D / 2);
    const vec vB(+W / 2, B, -D / 2);
    const vec vC(+W / 2, B, +D / 2);
    const vec vD(-W / 2, B, +D / 2);
    const vec vE(-W / 2, T, -D / 2);
    const vec vF(+W / 2, T, -D / 2);

    m_floor_tris.push_back(math::Triangle(vB, vA, vD));
    m_floor_tris.push_back(math::Triangle(vB, vD, vC));
    m_floor_tris.push_back(math::Triangle(vF, vE, vA));
    m_floor_tris.push_back(math::Triangle(vF, vA, vB));
}

void Ground::draw() const
{
    for (int i = 0; i < m_floor_tris.size(); i++)
    {
        vvr::Triangle3D floor_tri = vvr::math2vvr(m_floor_tris.at(i), m_col);
        floor_tri.setSolidRender(true);
        floor_tri.draw();
    }
}
