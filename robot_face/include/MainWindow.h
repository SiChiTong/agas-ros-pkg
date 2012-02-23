/*******************************************************************************
 *  TalkingHead - A talking head for robots
 *  Copyright (C) 2012 AG Aktives Sehen <agas@uni-koblenz.de>
 *                     Universitaet Koblenz-Landau
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA or see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

#ifndef TALKING_HEAD_INCLUDE_MAINWINDOW_H_
#define TALKING_HEAD_INCLUDE_MAINWINDOW_H_

#include <QWidget>

#include <ros/ros.h>

#include "TalkingHead.h"
#include "TextOutDisplay.h"
#include "FestivalGenerator.h"

/**
 * @class  MainWindow
 * @brief  Controls and displays Widgets
 * @author Julian Giesen (R16)(R18)
 */

class MainWindow : public QWidget
{
    Q_OBJECT

public:

    /** Constructor */
    explicit MainWindow( QWidget* parent = 0 );

    /** Destructor */
    virtual ~MainWindow();

    void setNodeHandle(ros::NodeHandle* node_handle);

private slots:

private:

    TalkingHead*            talking_head_;

    TextOutDisplay*         text_out_display_;
    TextOutDisplay*         text_rec_display_;

    FestivalGenerator*      festival_generator_;

};

#endif // TALKING_HEAD_INCLUDE_MAINWINDOW_H_
