/*******************************************************************************
 *  PainterCheckBox.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef PainterCheckBox_H
#define PainterCheckBox_H

#include <QWidget>

#include "PainterPlugin.h"

#include <QCheckBox>

/**
 * @class  PainterCheckBox
 * @brief  Small widget that displays a checkbox to enable/disable one painter plugin
 * @author David Gossow (R12)
 */
class PainterCheckBox: public QWidget
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    PainterCheckBox ( PainterPlugin* plugin, bool visible = true );

    /** @brief The destructor */
    ~PainterCheckBox();

    void setChecked ( bool visible ) { m_CheckBox->setChecked ( visible ); }

  signals:

    void changed();

  public slots:

    void setVisible ( int state );

  private:

    PainterPlugin* m_Plugin;
    QCheckBox* m_CheckBox;

};

#endif
