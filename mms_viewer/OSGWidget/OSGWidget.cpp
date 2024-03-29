#include "OSGWidget.h"
#include "PickHandler.h"

#include <osg/Camera>

#include <osg/DisplaySettings>
#include <osg/Geode>
#include <osg/Material>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

#include <osgDB/WriteFile>

#include <osgGA/EventQueue>
#include <osgGA/TrackballManipulator>

#include <osgUtil/IntersectionVisitor>
#include <osgUtil/PolytopeIntersector>

#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#include <cassert>

#include <stdexcept>
#include <vector>

#include <QDebug>
#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>


#ifdef WITH_SELECTION_PROCESSING
QRect makeRectangle(const QPoint& first, const QPoint& second)
{
    // Relative to the first point, the second point may be in either one of the
    // four quadrants of an Euclidean coordinate system.
    //
    // We enumerate them in counter-clockwise order, starting from the lower-right
    // quadrant that corresponds to the default case:
    //
    //            |
    //       (3)  |  (4)
    //            |
    //     -------|-------
    //            |
    //       (2)  |  (1)
    //            |

    if(second.x() >= first.x() && second.y() >= first.y())
        return QRect(first, second);
    else if(second.x() < first.x() && second.y() >= first.y())
        return QRect(QPoint(second.x(), first.y()), QPoint(first.x(), second.y()));
    else if(second.x() < first.x() && second.y() < first.y())
        return QRect(second, first);
    else if(second.x() >= first.x() && second.y() < first.y())
        return QRect(QPoint(first.x(), second.y()), QPoint(second.x(), first.y()));

    // Should never reach that point...
    return QRect();
}
#endif

namespace osgWidget{
    void Viewer::setupThreading()
    {
        if(_threadingModel == SingleThreaded)
        {
            if(_threadsRunning)
                stopThreading();
        }
        else
        {
            if(!_threadsRunning)
                startThreading();
        }
    }
}
OSGWidget::OSGWidget(QWidget* parent,
                     Qt::WindowFlags f)
    : QOpenGLWidget(parent, f)
    , mGraphicsWindow(new osgViewer::GraphicsWindowEmbedded(this->x(),
                                                            this->y(),
                                                            this->width(),
                                                            this->height()))
    , mViewer(new osgWidget::Viewer)
    , mIsSelectionActive(false)
    , mIsSelectionFinished(true)
{

    float aspectRatio = static_cast<float>(this->width() / 2) / static_cast<float>(this->height());
    auto pixelRatio   = this->devicePixelRatio();

    osg::Camera* camera = new osg::Camera;
    camera->setViewport(0, 0, this->width() / 2 * pixelRatio, this->height() * pixelRatio);
    camera->setClearColor(osg::Vec4(0.f, 0.f, 1.f, 1.f));
    camera->setProjectionMatrixAsPerspective(30.f, aspectRatio, 1.f, 1000.f);
    camera->setGraphicsContext(mGraphicsWindow);

    mViewer->setCamera(camera);
    //mViewer->setSceneData(geode);
    mViewer->addEventHandler(new osgViewer::StatsHandler);
#ifdef WITH_PICK_HANDLER
    mViewer->addEventHandler(new PickHandler(this->devicePixelRatio()));
#endif
    mViewer->setCameraManipulator(new osgGA::TrackballManipulator);

    mViewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    mViewer->realize();

    // This ensures that the widget will receive keyboard events. This focus
    // policy is not set by default. The default, Qt::NoFocus, will result in
    // keyboard events that are ignored.
    this->setFocusPolicy(Qt::StrongFocus);
    this->setMinimumSize(100, 100);
    // Ensures that the widget receives mouse move events even though no
    // mouse button has been pressed. We require this in order to let the
    // graphics window switch viewports properly.
    this->setMouseTracking(true);
}

OSGWidget::~OSGWidget()
{
}

void OSGWidget::paintEvent(QPaintEvent* /* paintEvent */)
{
    this->makeCurrent();

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    this->paintGL();

#ifdef WITH_SELECTION_PROCESSING
    if(mIsSelectionActive && !mIsSelectionFinished)
    {
        painter.setPen(Qt::black);
        painter.setBrush(Qt::transparent);
        painter.drawRect(makeRectangle(mSelectionStart, mSelectionEnd));
    }
#endif

    painter.end();

    this->doneCurrent();
}

void OSGWidget::paintGL()
{
    mViewer->frame();
}

void OSGWidget::resizeGL(int width, int height)
{
    this->getEventQueue()->windowResize(this->x(), this->y(), width, height);
    mGraphicsWindow->resized(this->x(), this->y(), width, height);

    this->onResize(width, height);
}

void OSGWidget::keyPressEvent(QKeyEvent* event)
{
    QString keyString   = event->text();
    const char* keyData = keyString.toLocal8Bit().data();

    if(event->key() == Qt::Key_S)
    {
#ifdef WITH_SELECTION_PROCESSING
        mIsSelectionActive = !mIsSelectionActive;
#endif

        // Further processing is required for the statistics handler here, so we do
        // not return right away.
    }
    else if(event->key() == Qt::Key_D)
    {
        osgDB::writeNodeFile(*mViewer->getSceneData(),
                             "/tmp/sceneGraph.osg");

        return;
    }
    else if(event->key() == Qt::Key_H)
    {
        this->onHome();
        return;
    }

    this->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::keyReleaseEvent(QKeyEvent* event)
{
    QString keyString   = event->text();
    const char* keyData = keyString.toLocal8Bit().data();

    this->getEventQueue()->keyRelease(osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::mouseMoveEvent(QMouseEvent* event)
{
    // Note that we have to check the buttons mask in order to see whether the
    // left button has been pressed. A call to `button()` will only result in
    // `Qt::NoButton` for mouse move events.
    if(mIsSelectionActive && event->buttons() & Qt::LeftButton)
    {
        mSelectionEnd = event->pos();

        // Ensures that new paint events are created while the user moves the
        // mouse.
        this->update();
    }
    else
    {
        auto pixelRatio = this->devicePixelRatio();

        this->getEventQueue()->mouseMotion(static_cast<float>(event->x() * pixelRatio),
                                           static_cast<float>(event->y() * pixelRatio));
    }
}

void OSGWidget::mousePressEvent(QMouseEvent* event)
{
    // Selection processing
    if(mIsSelectionActive && event->button() == Qt::LeftButton)
    {
        mSelectionStart    = event->pos();
        mSelectionEnd      = mSelectionStart; // Deletes the old selection
        mIsSelectionFinished = false;           // As long as this is set, the rectangle will be drawn
    }

    // Normal processing
    else
    {
        // 1 = left mouse button
        // 2 = middle mouse button
        // 3 = right mouse button

        unsigned int button = 0;

        switch(event->button())
        {
        case Qt::LeftButton:
            button = 1;
            break;

        case Qt::MiddleButton:
            button = 2;
            break;

        case Qt::RightButton:
            button = 3;
            break;

        default:
            break;
        }

        auto pixelRatio = this->devicePixelRatio();

        this->getEventQueue()->mouseButtonPress(static_cast<float>(event->x() * pixelRatio),
                                                static_cast<float>(event->y() * pixelRatio),
                                                button);
    }
}

void OSGWidget::mouseReleaseEvent(QMouseEvent* event)
{
    // Selection processing: Store end position and obtain selected objects
    // through polytope intersection.
    if(mIsSelectionActive && event->button() == Qt::LeftButton)
    {
        mSelectionEnd      = event->pos();
        mIsSelectionFinished = true; // Will force the painter to stop drawing the
        // selection rectangle

        this->processSelection();
    }

    // Normal processing
    else
    {
        // 1 = left mouse button
        // 2 = middle mouse button
        // 3 = right mouse button

        unsigned int button = 0;

        switch(event->button())
        {
        case Qt::LeftButton:
            button = 1;
            break;

        case Qt::MiddleButton:
            button = 2;
            break;

        case Qt::RightButton:
            button = 3;
            break;

        default:
            break;
        }

        auto pixelRatio = this->devicePixelRatio();

        this->getEventQueue()->mouseButtonRelease(static_cast<float>(pixelRatio * event->x()),
                                                  static_cast<float>(pixelRatio * event->y()),
                                                  button);
    }
}

void OSGWidget::wheelEvent(QWheelEvent* event)
{
    // Ignore wheel events as long as the selection is active.
    if(mIsSelectionActive)
        return;

    event->accept();
    int delta = event->delta();

    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?   osgGA::GUIEventAdapter::SCROLL_UP
                                                                 : osgGA::GUIEventAdapter::SCROLL_DOWN;

    this->getEventQueue()->mouseScroll(motion);
}

bool OSGWidget::event(QEvent* event)
{
    bool handled = QOpenGLWidget::event(event);

    // This ensures that the OSG widget is always going to be repainted after the
    // user performed some interaction. Doing this in the event handler ensures
    // that we don't forget about some event and prevents duplicate code.
    switch(event->type())
    {
    case QEvent::KeyPress:
    case QEvent::KeyRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::Wheel:
        this->update();
        break;

    default:
        break;
    }

    return handled;
}

void OSGWidget::onHome()
{
    osgViewer::ViewerBase::Views views;
    mViewer->getViews(views);

    for(std::size_t i = 0; i < views.size(); i++)
    {
        osgViewer::View* view = views.at(i);
        view->home();
    }
}

void OSGWidget::onResize(int width, int height)
{
    auto pixelRatio = this->devicePixelRatio();

    mViewer->getCamera()->setViewport(0, 0, width * pixelRatio, height * pixelRatio);
}

osgGA::EventQueue* OSGWidget::getEventQueue() const
{
    osgGA::EventQueue* eventQueue = mGraphicsWindow->getEventQueue();

    if(eventQueue)
        return eventQueue;
    else
        throw std::runtime_error("Unable to obtain valid event queue");
}

void OSGWidget::processSelection()
{
#ifdef WITH_SELECTION_PROCESSING
    QRect selectionRectangle = makeRectangle(mSelectionStart, mSelectionEnd);
    auto widgetHeight        = this->height();
    auto pixelRatio          = this->devicePixelRatio();

    double xMin = selectionRectangle.left();
    double xMax = selectionRectangle.right();
    double yMin = widgetHeight - selectionRectangle.bottom();
    double yMax = widgetHeight - selectionRectangle.top();

    xMin *= pixelRatio;
    yMin *= pixelRatio;
    xMax *= pixelRatio;
    yMax *= pixelRatio;

    osgUtil::PolytopeIntersector* polytopeIntersector
            = new osgUtil::PolytopeIntersector(osgUtil::PolytopeIntersector::WINDOW,
                                               xMin, yMin,
                                               xMax, yMax);

    // This limits the amount of intersections that are reported by the
    // polytope intersector. Using this setting, a single drawable will
    // appear at most once while calculating intersections. This is the
    // preferred and expected behaviour.
    polytopeIntersector->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE_PER_DRAWABLE);

    osgUtil::IntersectionVisitor iv(polytopeIntersector);

    if(!mViewer)
        throw std::runtime_error("Unable to obtain valid view for selection processing");

    osg::Camera* camera = mViewer->getCamera();

    if(!camera)
        throw std::runtime_error("Unable to obtain valid camera for selection processing");

    camera->accept(iv);

    if(!polytopeIntersector->containsIntersections())
        return;

    auto intersections = polytopeIntersector->getIntersections();

    for(auto&& intersection : intersections)
        qDebug() << "Selected a drawable:" << QString::fromStdString(intersection.drawable->getName());

#endif
}
