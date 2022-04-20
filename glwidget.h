#ifndef GLWIDGET_H_
#define GLWIDGET_H_

#include <QGLWidget>
// The factory
#include <GLC_Factory>
// Light
#include <GLC_Light>
// The Viewport with a default camera
#include <GLC_Viewport>
// The world which manage product structure
#include <GLC_World>
// The Mover controller is used to change the point of view
#include <GLC_MoverController>

namespace mag {
struct axisAngle{
    float angle;
    float x;
    float y;
    float z;
};

struct Quaternion{
    double q0;
    double q1;
    double q2;
    double q3;
};

inline axisAngle qtoAxisAngle(double q0, double q1, double q2, double q3){
    double qNorm = (q0*q0 + q1*q1 + q2*q2 + q3*q3);
    double x = 2*(-q0*q2+q1*q3)/qNorm;
    double y = 2*(q0*q1+q2*q3)/qNorm;
    double z = (q0*q0 - q1*q1 - q2*q2 + q3*q3)/qNorm;
    axisAngle magInfo;

    magInfo.angle = 180*acos(z)/3.14159;
    magInfo.x = -y/sqrt(x*x+y*y);
    magInfo.y = x/sqrt(x*x+y*y);
    magInfo.z = 0;
    return magInfo;
}
}


class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = nullptr);
    ~GLWidget();
    void updateCapsuleState(double a, double b, double c, double w,
                            double x, double y, double z);
    void showCapsuleAngle(double yaw, double pitch, double roll);
    void setAcFilename(QString acf);
    void setBgFilename(QString bgf);
    void setVboEnable(bool eVbo);
    void reloadScene();

private:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    // Create GLC_Object to display
    void CreateScene();

    double m_posX, m_posY, m_posZ;
    double m_yaw, m_pitch, m_roll;

    //create sub-scene
    GLC_World loadCapsule();
    void CreateAxis(GLC_StructOccurrence* pRoot);
    void CreateGrid(GLC_StructOccurrence* pRoot);

    //Mouse events
    void mousePressEvent(QMouseEvent * e);
    void mouseMoveEvent(QMouseEvent * e);
    void mouseReleaseEvent(QMouseEvent * e);

private:
    GLC_Light m_Light;
    QColor m_DefaultColor;
    GLC_World m_World;
    GLC_Viewport m_GlView;
    GLC_MoverController m_MoverController;

    QString m_acFilename;
    QString m_bgFilename;
    bool m_vboEnable;
};

#endif /*GLWIDGET_H_*/
