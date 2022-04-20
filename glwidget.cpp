#include <QtDebug>
#include <QGLContext>
#include <GLC_Octree>
#include <GLC_Context>
#include <GLC_Polylines>
#include <QApplication>

#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent)
: QGLWidget(new QGLContext(QGLFormat(QGL::SampleBuffers)), parent)
, m_Light()
, m_DefaultColor()
, m_World()
, m_GlView()
, m_MoverController(),
  m_acFilename(),
  m_bgFilename(),
  m_vboEnable(false),
  m_posX(0),
  m_posY(0),
  m_posZ(0),
  m_yaw(0),
  m_pitch(0),
  m_roll(0)
{

    connect(&m_GlView, SIGNAL(updateOpenGL()), this, SLOT(updateGL()));
    m_Light.setPosition(1.0, 1.0, 1.0);

    m_DefaultColor.setRgbF(0.5, 0.8, 1.0, 1.0);

    m_GlView.cameraHandle()->setDefaultUpVector(glc::Z_AXIS);
    m_GlView.cameraHandle()->setRearView();
    GLC_Point3d myView;
    myView.setVect(5, -15, -9);
    m_GlView.cameraHandle()->setTargetCam(myView);



    QColor repColor;
    repColor.setRgbF(1.0, 0.11372, 0.11372, 1.0);
    m_MoverController= GLC_Factory::instance()->createDefaultMoverController(repColor, &m_GlView);

    // Create objects to display
    CreateScene();
}

GLWidget::~GLWidget()
{

}

void GLWidget::initializeGL()
{
    // OpenGL initialisation from NEHE production
    m_GlView.initGl();
    m_GlView.loadBackGroundImage(":Models/background.png");
    m_GlView.reframe(m_World.boundingBox());

    glEnable(GL_NORMALIZE);
    // Enable antialiasing
    glEnable(GL_MULTISAMPLE);

    glEnable(GL_LINE_SMOOTH);//抗锯齿启用
    glHint(GL_LINE_SMOOTH, GL_NICEST);
}

void GLWidget::paintGL()
{
    // Clear screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Load identity matrix
    GLC_Context::current()->glcLoadIdentity();

    m_GlView.setDistMinAndMax(m_World.boundingBox());

    // define the light
    m_Light.glExecute();

    // define view matrix
    m_GlView.glExecuteCam();

    // Display the collection of GLC_Object
    m_World.render(0, glc::ShadingFlag);
    m_World.render(0, glc::TransparentRenderFlag);

    qglColor(Qt::red);
    QString du = QString::fromLocal8Bit("°");
    renderText(20, 40, "Yaw:   " + QString::number(m_yaw * 57.3, 'f', 2) + du,
               QFont("Times New Roman", 14, QFont::Bold));
    renderText(20, 70, "Pitch: " + QString::number(m_pitch * 57.3, 'f', 2) + du,
               QFont("Times New Roman", 14, QFont::Bold));
    renderText(20, 100, "Roll:   " + QString::number(m_roll * 57.3, 'f', 2) + du,
               QFont("Times New Roman", 14, QFont::Bold));

    renderText(this->width()-90, 40, "X: " + QString::number(m_posX,'f', 2) + "cm",
               QFont("Times New Roman", 14, QFont::Bold));
    qglColor(Qt::green);
    renderText(this->width()-90, 70, "Y: " + QString::number(m_posY,'f', 2) + "cm",
               QFont("Times New Roman", 14, QFont::Bold));
    qglColor(Qt::color0);
    renderText(this->width()-90, 100, "Z: " + QString::number(m_posZ,'f', 2) + "cm",
               QFont("Times New Roman", 14, QFont::Bold));

    // Display UI Info (orbit circle)
    m_MoverController.drawActiveMoverRep();
}

void GLWidget::resizeGL(int width, int height)
{
    m_GlView.setWinGLSize(width, height);	// Compute window aspect ratio
}

// Create GLC_Object to display
void GLWidget::CreateScene()
{
    m_World= loadCapsule();
    GLC_StructOccurrence* pRoot= m_World.rootOccurrence();
    CreateAxis(pRoot);
    CreateGrid(pRoot);
}

GLC_World GLWidget::loadCapsule(){
    QString path = QApplication::applicationDirPath();  //lib\share\gcs\models\capsule
    //QString path1 = path.left(path.lastIndexOf("/"));
    QString modelFile = path + "/models/capsule/capsule.3DS";
    qDebug()<<__FUNCTION__<<"----PATH:"<<modelFile;
    QFileInfo file(modelFile);
    if(!file.exists()){
        QMessageBox::warning(NULL, QStringLiteral("警告"), QStringLiteral("capsule.3DS文件未找到!") + "\n Path:" + modelFile);
    }
    QFile ObjFile(modelFile);
    return GLC_Factory::instance()->createWorldFromFile(ObjFile);
}

void GLWidget::CreateAxis(GLC_StructOccurrence *pRoot){
    GLC_StructInstance* pInstance= NULL;
    GLC_Vector3d movXYZ;
    // create Z axis representation
    GLC_3DRep cylinder(GLC_Factory::instance()->createCylinder(1, 100));
    GLC_3DRep cone(GLC_Factory::instance()->createCone(3,8));
    cylinder.geomAt(0)->replaceMasterMaterial(new GLC_Material(Qt::color0));
    cone.geomAt(0)->replaceMasterMaterial(new GLC_Material(Qt::color0));
    GLC_Matrix4x4 matRot(glc::Z_AXIS, 0);			//Create a rotation matrix
    pInstance= new GLC_StructInstance(cylinder.clone());
    pInstance->move(matRot);								// move the cylinder
    pRoot->addChild(pInstance);
    pInstance= new GLC_StructInstance(cone.clone());
    pInstance->move(matRot);								// move the cylinder
    movXYZ.setVect(0, 0, 100);
    pInstance->translate(movXYZ);
    pRoot->addChild(pInstance);


    // create X axis representation
    cylinder= GLC_Factory::instance()->createCylinder(1, 300);
    cylinder.geomAt(0)->replaceMasterMaterial(new GLC_Material(Qt::red));
    cone = GLC_Factory::instance()->createCone(3,8);
    cone.geomAt(0)->replaceMasterMaterial(new GLC_Material(Qt::red));
    matRot.setMatRot(glc::Y_AXIS, -glc::PI/2);				//Create a rotation matrix
    pInstance= new GLC_StructInstance(cylinder.clone());
    pInstance->move(matRot);								// move the cylinder
    movXYZ.setVect(150, 0, 0);
    pInstance->translate(movXYZ);
    pRoot->addChild(pInstance);
    pInstance= new GLC_StructInstance(cone.clone());
    pInstance->move(matRot);								// move the cylinder
    movXYZ.setVect(-150, 0, 0);
    pInstance->translate(movXYZ);
    pRoot->addChild(pInstance);


    // create Y axis representation
    cylinder= GLC_Factory::instance()->createCylinder(1, 300);
    cylinder.geomAt(0)->replaceMasterMaterial(new GLC_Material(Qt::green));
    cone = GLC_Factory::instance()->createCone(3,8);
    cone.geomAt(0)->replaceMasterMaterial(new GLC_Material(Qt::green));
    matRot.setMatRot(glc::X_AXIS, glc::PI/2);								// Set rotation matrix
    pInstance= new GLC_StructInstance(cylinder.clone());
    pInstance->move(matRot);								// move the cylinder
    movXYZ.setVect(0, 150, 0);
    pInstance->translate(movXYZ);
    pRoot->addChild(pInstance);
    pInstance= new GLC_StructInstance(cone.clone());
    pInstance->move(matRot);								// move the cylinder
    movXYZ.setVect(0, -150, 0);
    pInstance->translate(movXYZ);
    pRoot->addChild(pInstance);
}

void GLWidget::CreateGrid(GLC_StructOccurrence *pRoot){
    int size = 300;
    for(int i=0; i<(size+1);i+=20){
        GLC_3DRep xline(GLC_Factory::instance()->createLine(GLC_Point3d(i-size/2, -size/2, 0),GLC_Point3d(i-size/2, size/2, 0)));
        GLC_3DRep yline(GLC_Factory::instance()->createLine(GLC_Point3d(-size/2, i-size/2, 0),GLC_Point3d(size/2, i-size/2, 0)));
        xline.geomAt(0)->setWireColor(QColor(210, 210, 210));
        yline.geomAt(0)->setWireColor(QColor(210, 210, 210));
        xline.geomAt(0)->setLineWidth(0.1f);
        yline.geomAt(0)->setLineWidth(0.1f);
        pRoot->addChild(new GLC_StructInstance(xline.clone()));
        pRoot->addChild(new GLC_StructInstance(yline.clone()));
    }

}

void GLWidget::updateCapsuleState(double a, double b, double c,
                                  double w, double x, double y, double z)
{
    m_posX = -a;
    m_posY = -b;
    m_posZ = c;

    GLC_StructOccurrence* pRoot= m_World.rootOccurrence();

    QMatrix4x4 m1;  // 这是定义四元数的左乘？与标准的不一致！
    m1.setRow(0, QVector4D(w, z, -y, x));
    m1.setRow(1, QVector4D(-z, w, x, y));
    m1.setRow(2, QVector4D(y, -x, w, z));
    m1.setRow(3, QVector4D(-x, -y, -z, w));
    QMatrix4x4 m2;
    m2.setRow(0, QVector4D(w, z, -y, -x));
    m2.setRow(1, QVector4D(-z, w, x, -y));
    m2.setRow(2, QVector4D(y, -x, w, -z));
    m2.setRow(3, QVector4D(x, y, z, w));

    QMatrix4x4 m0;
    m0 = m1 * m2;
    GLC_Matrix4x4 rootObjectRotation(m0.data());

    for(int i= 0; i<3;i++){
        pRoot->child(i)->structInstance()->setMatrix(rootObjectRotation);
        pRoot->child(i)->structInstance()->translate(10*a, 10*b, 10*c);
    }
    pRoot->updateChildrenAbsoluteMatrix();
    updateGL();
}

void GLWidget::showCapsuleAngle(double yaw, double pitch, double roll)
{
    m_yaw   = yaw;
    m_pitch = pitch;
    m_roll  = roll;
}

void GLWidget::mousePressEvent(QMouseEvent *e)
{
    if (m_MoverController.hasActiveMover()) return;
    switch (e->button())
    {
    case (Qt::RightButton):
        m_MoverController.setActiveMover(GLC_MoverController::TrackBall, GLC_UserInput(e->x(), e->y()));
        updateGL();
        break;
    case (Qt::LeftButton):
        m_MoverController.setActiveMover(GLC_MoverController::Pan, GLC_UserInput(e->x(), e->y()));
        updateGL();
        break;
    case (Qt::MidButton):
        m_MoverController.setActiveMover(GLC_MoverController::Zoom, GLC_UserInput(e->x(), e->y()));
        updateGL();
        break;

    default:
        break;
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent * e)
{
    if (m_MoverController.hasActiveMover())
    {
        m_MoverController.move(GLC_UserInput(e->x(), e->y()));
        m_GlView.setDistMinAndMax(m_World.boundingBox());
        updateGL();
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *)
{
    if (m_MoverController.hasActiveMover())
    {
        m_MoverController.setNoMover();
        updateGL();
    }
}

void GLWidget::setAcFilename(QString acf)
{
    qDebug()<<__FUNCTION__ << "---file0 " << m_acFilename <<acf;
    if (QFile::exists(acf)) {
        m_acFilename = acf;
    } else {
        m_acFilename = ":/models/warning_sign.obj";
        m_GlView.cameraHandle()->setFrontView(); // set to front camera to see/read the warning sign
    }
    qDebug()<<__FUNCTION__ << "---file " << m_acFilename;
}

void GLWidget::setBgFilename(QString bgf)
{
    if (QFile::exists(bgf)) {
        m_bgFilename = bgf;
    } else {
        qDebug()<<__FUNCTION__ << "file " << bgf << " doesn't exists";
        // will put a black background if there's no background
        m_bgFilename = ":/models/black.jpg";
    }
}

void GLWidget::setVboEnable(bool eVbo)
{
    m_vboEnable = eVbo;
    m_World.collection()->setVboUsage(m_vboEnable);
}

void GLWidget::reloadScene()
{
    CreateScene();
}
