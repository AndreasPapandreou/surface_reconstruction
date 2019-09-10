#ifndef VVR_GLWIDGET_H
#define VVR_GLWIDGET_H

#include "scene.h"
#include <QtOpenGL>
#include <QGLWidget>
#include <QInputEvent>

namespace vvr {

class VVRScene_API GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(vvr::Scene *scene, QWidget *parent = 0);
    ~GLWidget();

public slots:
    void onKeyPressed(QKeyEvent *event);
    void idle();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);

private: //data
    vvr::Scene *mScene;
    QTimer timer;

private:
    static int mkModif(QInputEvent *event);

};

}

#endif // VVR_GLWIDGET_H
