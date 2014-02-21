#include "rosprocessbutton.h"
#include <QtCore>
#include <QPushButton>
#include <iostream>
#include <QContextMenuEvent>
#include <QMenu>

RosProcessButton::RosProcessButton(QWidget *parent = 0):
    QPushButton(parent)
{
}

void RosProcessButton::contextMenuEvent ( QContextMenuEvent * event )
{
    QMenu menu(this);
    menu.addAction("Start in terminal");
    menu.addAction("Kill process");


    connect(menu.actions()[0], SIGNAL(triggered()), this, SIGNAL(startInTerminalPushed()));
    connect(menu.actions()[1], SIGNAL(triggered()), this, SIGNAL(killPushed()));
    menu.exec(event->globalPos());
}
