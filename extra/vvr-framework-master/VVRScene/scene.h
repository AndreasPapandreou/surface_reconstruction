#ifndef VVR_SCENE_H
#define VVR_SCENE_H

//! MACROS used for toggling and testing bitwise flags.
#define VVR_FLAG(x) (1<<(x))
#define VVR_FLAG_ON(v,f) (v & VVR_FLAG(f))
#define VVR_FLAG_CASE_TOGGLE(v,c,f) case c: v ^= VVR_FLAG(f); std::cout \
    << #f << " = " << (VVR_FLAG_ON(v,f) ? "ON" : "OFF") \
    << std::endl; break

#include "vvrscenedll.h"
#include "canvas.h"
#include <MathGeoLib.h>

namespace vvr {

    struct IAnimatable
    {
        virtual bool animate(float time) = 0;
        virtual float getTotalDuration() { return 0; }
    };

    class Animation
    {
    public:
        const float &t;

        Animation()
            : t(m_time)
            , m_time(0)
            , m_speed(1)
            , m_paused(true)
        {

        }

        void pause() { m_paused = true; }
        bool paused() const { return m_paused; }
        void setSpeed(float speed) { m_speed = speed; }
        float speed() { return m_speed; }

        void update(bool force_resume = false)
        {
            const float sec = getSeconds();
            if (m_paused) if (force_resume) m_last_update = sec; else return;
            m_paused = false;
            m_time += ((sec - m_last_update) * m_speed);
            m_last_update = sec;
        }

        void setTime(float time)
        {
            const float sec = getSeconds();
            m_time = time;
            m_last_update = sec;
        }

    private:
        bool m_paused;
        float m_time;
        float m_last_update;
        float m_end_time;
        float m_speed;
    };

    enum ArrowDir
    {
        UP = 0,
        DOWN,
        RIGHT,
        LEFT
    };

    /**
     * Class Representing scene floor and background wall.
     */
    class VVRScene_API Ground : public vvr::IRenderable
    {
    public:
        Ground(const float W, const float D, const float B, const float T, const vvr::Colour &colour);
        void draw() const override;

    private:
        std::vector<math::Triangle> m_floor_tris;
        vvr::Colour m_col;
    };

    class VVRScene_API Scene
    {
    private:
        Frustum m_frustum;
        float m_fov;
        float m_camera_dist;
        float m_scene_width, m_scene_height;
        int m_screen_width, m_screen_height;
        int m_mouselastX, m_mouselastY;

    protected:
        Colour m_bg_col;
        bool m_perspective_proj;
        bool m_fullscreen;
        bool m_create_menus;
        bool m_hide_log;
        bool m_hide_sliders;

    protected:
        virtual void draw() = 0;
        virtual void reset();
        virtual void resize(){}

    protected:
        void drawAxes();
        void enterPixelMode();
        void returnFromPixelMode();

    public:
        Scene();
        virtual ~Scene(){}

        virtual const char* getName() const {
            return "VVR Framework Scene";
        }

        //! OpenGL Callbacks

        void GL_Render();
        void GL_Init();
        void GL_Resize(int width, int height);

        //! Events

        virtual bool idle(){ return false; }
        virtual void keyEvent(unsigned char key, bool up, int modif);
        virtual void arrowEvent(ArrowDir dir, int modif);
        virtual void mousePressed(int x, int y, int modif);
        virtual void mouseReleased(int x, int y, int modif);
        virtual void mouseMoved(int x, int y, int modif);
        virtual void mouseWheel(int dir, int modif);
        virtual void sliderChanged(int slider_id, float val);

        //! Getters

        int getViewportWidth() { return m_screen_width; } // In pixels
        int getViewportHeight() { return m_screen_height; } // In pixels
        float getSceneWidth() { return m_scene_width; }
        float getSceneHeight() { return m_scene_height; }
        Frustum getFrustum() { return m_frustum; };
        bool fullScreen() { return m_fullscreen; }
        bool createMenus() { return m_create_menus; }
        bool hideLog() { return m_hide_log; }
        bool hideSliders() { return m_hide_sliders; }

        //! Setters

        void setFrustum(const Frustum &frustum) { m_frustum = frustum; }
        void setCol(const Colour& col) { m_bg_col = col; }
        void setSliderVal(int slider_id, float val);

    public:

        void setCameraPos(const vec &pos);

        //! Helpers

    public:
        void mouse2pix(int &x, int &y);
        Ray unproject(int x, int y);
        bool ctrlDown(int modif) { return modif & (1 << 0); }
        bool shiftDown(int modif) { return modif & (1 << 1); }
        bool altDown(int modif) { return modif & (1 << 2); }
    };

    int VVRScene_API mainLoop(int argc, char* argv[], Scene *scene);

}

#endif // VVR_SCENE_H
