/********************************************************************************
** Form generated from reading UI file 'window.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WINDOW_H
#define UI_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_left;
    QPlainTextEdit *plain_text_log;
    QGroupBox *slider_groupbox;
    QVBoxLayout *verticalLayout_3;
    QSlider *horizontalSlider_0;
    QSlider *horizontalSlider_1;
    QSlider *horizontalSlider_2;
    QSlider *horizontalSlider_3;
    QSlider *horizontalSlider_4;
    QSlider *horizontalSlider_5;
    QSpacerItem *verticalSpacer;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(812, 468);
        MainWindow->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid grey;\n"
"    border-radius: 4px;\n"
"    margin-top: 0.5em;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 10px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
""));
        MainWindow->setUnifiedTitleAndToolBarOnMac(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(2, 2, 2, 2);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout_left = new QVBoxLayout();
        verticalLayout_left->setSpacing(6);
        verticalLayout_left->setObjectName(QStringLiteral("verticalLayout_left"));
        verticalLayout_left->setContentsMargins(3, -1, -1, 0);
        plain_text_log = new QPlainTextEdit(centralWidget);
        plain_text_log->setObjectName(QStringLiteral("plain_text_log"));
        plain_text_log->setStyleSheet(QLatin1String("QPlainTextEdit { \n"
"    border: 1px solid grey;\n"
"    border-radius: 4px;\n"
"    font: Ubuntu mono;\n"
"    font-size: 12px;\n"
"	background-color: #333;\n"
"    color: #00aa33;\n"
"}\n"
""));
        plain_text_log->setReadOnly(true);
        plain_text_log->setTextInteractionFlags(Qt::TextSelectableByMouse);

        verticalLayout_left->addWidget(plain_text_log);

        slider_groupbox = new QGroupBox(centralWidget);
        slider_groupbox->setObjectName(QStringLiteral("slider_groupbox"));
        verticalLayout_3 = new QVBoxLayout(slider_groupbox);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        horizontalSlider_0 = new QSlider(slider_groupbox);
        horizontalSlider_0->setObjectName(QStringLiteral("horizontalSlider_0"));
        horizontalSlider_0->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(horizontalSlider_0);

        horizontalSlider_1 = new QSlider(slider_groupbox);
        horizontalSlider_1->setObjectName(QStringLiteral("horizontalSlider_1"));
        horizontalSlider_1->setMaximum(100);
        horizontalSlider_1->setOrientation(Qt::Horizontal);
        horizontalSlider_1->setTickPosition(QSlider::NoTicks);

        verticalLayout_3->addWidget(horizontalSlider_1);

        horizontalSlider_2 = new QSlider(slider_groupbox);
        horizontalSlider_2->setObjectName(QStringLiteral("horizontalSlider_2"));
        horizontalSlider_2->setMaximum(100);
        horizontalSlider_2->setOrientation(Qt::Horizontal);
        horizontalSlider_2->setTickPosition(QSlider::NoTicks);

        verticalLayout_3->addWidget(horizontalSlider_2);

        horizontalSlider_3 = new QSlider(slider_groupbox);
        horizontalSlider_3->setObjectName(QStringLiteral("horizontalSlider_3"));
        horizontalSlider_3->setMaximum(100);
        horizontalSlider_3->setOrientation(Qt::Horizontal);
        horizontalSlider_3->setTickPosition(QSlider::NoTicks);

        verticalLayout_3->addWidget(horizontalSlider_3);

        horizontalSlider_4 = new QSlider(slider_groupbox);
        horizontalSlider_4->setObjectName(QStringLiteral("horizontalSlider_4"));
        horizontalSlider_4->setMaximum(100);
        horizontalSlider_4->setOrientation(Qt::Horizontal);
        horizontalSlider_4->setTickPosition(QSlider::NoTicks);

        verticalLayout_3->addWidget(horizontalSlider_4);

        horizontalSlider_5 = new QSlider(slider_groupbox);
        horizontalSlider_5->setObjectName(QStringLiteral("horizontalSlider_5"));
        horizontalSlider_5->setMaximum(100);
        horizontalSlider_5->setOrientation(Qt::Horizontal);
        horizontalSlider_5->setTickPosition(QSlider::NoTicks);

        verticalLayout_3->addWidget(horizontalSlider_5);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        horizontalSlider_4->raise();
        horizontalSlider_2->raise();
        horizontalSlider_5->raise();
        horizontalSlider_3->raise();
        horizontalSlider_1->raise();
        horizontalSlider_0->raise();

        verticalLayout_left->addWidget(slider_groupbox, 0, Qt::AlignTop);

        verticalLayout_left->setStretch(0, 10);
        verticalLayout_left->setStretch(1, 1);

        horizontalLayout->addLayout(verticalLayout_left);

        scrollArea = new QScrollArea(centralWidget);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 538, 460));
        scrollArea->setWidget(scrollAreaWidgetContents);

        horizontalLayout->addWidget(scrollArea);

        horizontalLayout->setStretch(1, 5);

        verticalLayout->addLayout(horizontalLayout);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        slider_groupbox->setTitle(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WINDOW_H
