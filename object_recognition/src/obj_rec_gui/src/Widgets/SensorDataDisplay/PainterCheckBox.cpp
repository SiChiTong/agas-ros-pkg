/*******************************************************************************
 *  PainterCheckBox.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "PainterCheckBox.h"

#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>

#define THIS PainterCheckBox

THIS::THIS ( PainterPlugin* plugin, bool visible ) : QWidget( )
{
  m_Plugin = plugin;
  m_CheckBox = new QCheckBox ( plugin->getName().c_str() );

  QVBoxLayout* layout = new QVBoxLayout ( this );
  layout->addWidget ( m_CheckBox );
  layout->setMargin ( 0 );
  layout->setSpacing ( 0 );
  setLayout ( layout );

  connect ( m_CheckBox , SIGNAL ( stateChanged ( int ) ), this, SLOT ( setVisible ( int ) ) );
  m_CheckBox->setChecked ( visible );
}

THIS::~THIS()
{
}

void THIS::setVisible ( int state )
{
  m_Plugin->setVisible ( state == Qt::Checked );
  emit changed();
}

#undef THIS
