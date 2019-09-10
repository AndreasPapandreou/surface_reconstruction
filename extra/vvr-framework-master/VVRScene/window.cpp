#include "window.h"
#include "glwidget.h"
#include "scene.h"
#include "logger.h"
#include <QtOpenGL>
#include <QtWidgets>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QDir>
#include <iostream>

using std::cerr;
using std::endl;

QString vvr::Window::aboutMessage = QString("VVR LAB 2016") + QString(QChar(0xA9));

vvr::Window::Window(vvr::Scene *scene) : scene(scene)
{
    setupUi(this);
    setWindowTitle(tr(scene->getName()));

    // Redirect std::cout to our custom logging widget
    m_std_cout_logger = new StdRedirector<>(std::cout, &Window::s_log_cout, this);
    m_std_cerr_logger = new StdRedirector<>(std::cerr, &Window::s_log_cerr, this);
    connect(this, SIGNAL(log_cout(const QString&)), this, SLOT(do_log_cout(const QString&)));
    connect(this, SIGNAL(log_cerr(const QString&)), this, SLOT(do_log_cerr(const QString&)));

    // Init glwidget
    glWidget = new vvr::GLWidget(scene);
    scrollArea->setWidget(glWidget);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    connect(this, SIGNAL(keyPressed(QKeyEvent*)), glWidget, SLOT(onKeyPressed(QKeyEvent*)));
    connect(horizontalSlider_0, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));
    connect(horizontalSlider_1, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));
    connect(horizontalSlider_2, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));
    connect(horizontalSlider_3, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));
    connect(horizontalSlider_4, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));
    connect(horizontalSlider_5, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));
    createActions();

    if (scene->createMenus()) {
        createMenus();
    }
    if (scene->hideLog()) {
        plain_text_log->hide();
    }
    if (scene->hideSliders()) {
        QLayout * layout = slider_groupbox->layout();
        QLayoutItem * item;
        QLayout * sublayout;
        QWidget * widget;
        while ((item = layout->takeAt(0))) {
            if ((sublayout = item->layout()) != 0) {/* do the same for sublayout*/}
            else if ((widget = item->widget()) != 0) { widget->hide(); delete widget; }
            else { delete item; }
        }
        delete layout;
        delete slider_groupbox;
    }

    if (scene->fullScreen()) {
        QTimer::singleShot(150, this, SLOT(showFullScreen()));
    }
    else {
        showNormal();
    }

    glWidget->setFocus();
}

void vvr::Window::createActions()
{
    // exit action
    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    // about action
    aboutAct = new QAction(tr("&About"), this);
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
}

void vvr::Window::createMenus()
{
    // file menu
    fileMenu = QMainWindow::menuBar()->addMenu(tr("&File"));
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    // help menu
    helpMenu = QMainWindow::menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
}

void vvr::Window::about()
{
    QMessageBox::about(this, scene->getName(), aboutMessage);
}

void vvr::Window::sliderMoved(int val)
{
    const int id = std::stoi(vvr::split(sender()->objectName().toStdString(), '_').back());
    scene->sliderChanged(id, val / 100.0f);
    glWidget->update();
}

void vvr::Window::keyPressEvent(QKeyEvent* event)
{
    std::string str = event->text().toStdString();
    if (str.length() > 0) emit keyPressed(event);
}

//! Console output redirecting

void vvr::Window::s_log_cout(const char* ptr, std::streamsize count, void* pte)
{
    printf("%.*s", (int)count, ptr);
    vvr::Window *window = static_cast<vvr::Window*>(pte);
    const QString str = QString::fromLocal8Bit(ptr, count);
    window->log_cout(str);
}

void vvr::Window::s_log_cerr(const char* ptr, std::streamsize count, void* pte)
{
    fprintf(stderr, "%.*s", (int)count, ptr);
    vvr::Window *window = static_cast<vvr::Window*>(pte);
    const QString str = QString::fromLocal8Bit(ptr, count);
    window->log_cerr(str);
}

void vvr::Window::do_log_cout(const QString &str)
{
    QScrollBar *vScrollBar = plain_text_log->verticalScrollBar();
    const bool keep_on_bottom = vScrollBar->value() == vScrollBar->maximum();

    //! Print text
    plain_text_log->moveCursor(QTextCursor::End);
    QString html = str.toHtmlEscaped();
    html.replace(QChar(' '), QString("&nbsp;"), Qt::CaseInsensitive);
    plain_text_log->textCursor().insertHtml("<font color=\"White\">" + html + "</font>");
    plain_text_log->moveCursor(QTextCursor::End);

    //! New line
    if (str.at(str.length() - 1) == '\n') {
        plain_text_log->appendPlainText("");
    }

    if (keep_on_bottom) {
        vScrollBar->triggerAction(QScrollBar::SliderToMaximum);
    }

    //! Optionally log
#ifdef VVR_USE_BOOST
    vvr::logi(str.toStdString());
#endif
}

void vvr::Window::do_log_cerr(const QString &str)
{
    QScrollBar *vScrollBar = plain_text_log->verticalScrollBar();
    const bool keep_on_bottom = vScrollBar->value() == vScrollBar->maximum();

    //! Print text
    QString html = str.toHtmlEscaped();
    html.replace(QChar(' '), QString("&nbsp;"), Qt::CaseInsensitive);
    plain_text_log->moveCursor(QTextCursor::End);
    plain_text_log->textCursor().insertHtml("<font color=\"Red\">" + html + "</font>");
    plain_text_log->moveCursor(QTextCursor::End);

    //! New line
    if (str.at(str.length() - 1) == '\n') {
        plain_text_log->appendPlainText("");
    }

    if (keep_on_bottom) {
        vScrollBar->triggerAction(QScrollBar::SliderToMaximum);
    }

    //! Optionally log
#ifdef VVR_USE_BOOST
    vvr::loge(str.toStdString());
#endif
}

int vvr::mainLoop(int argc, char* argv[], vvr::Scene *scene)
{
    QApplication app(argc, argv);
    QPixmap pixmap(":/Icons/vvrframework-splash.png");
    QSplashScreen splash(pixmap);
    splash.show();
    app.processEvents();
    Window window(scene);
    window.show();
    splash.close();
    app.exec();
    return 0;
}
