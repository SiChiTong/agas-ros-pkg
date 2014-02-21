/*******************************************************************************
 *  PainterPlugin.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "PainterPlugin.h"

#define THIS PainterPlugin

THIS::THIS() : QObject()
{
  m_Visible = true;
  m_Name = "Unnamed Painter Plugin";
  m_NeedsRedraw = false;
  m_ParentWidget = 0;
}

THIS::~THIS()
{
}

#undef THIS
