/****************************************************************************

 This file is part of the GLC-lib library.
 Copyright (C) 2005-2008 Laurent Ribon (laumaya@users.sourceforge.net)
 http://glc-lib.sourceforge.net

 GLC-lib is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 GLC-lib is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with GLC-lib; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

 *****************************************************************************/
#include <QMouseEvent>
#include <QTouchEvent>
#include <QWheelEvent>

#include <QtDebug>

#include "glc_viewhandler.h"
#include "glc_inputeventinterpreter.h"


GLC_InputEventInterpreter::GLC_InputEventInterpreter(GLC_ViewHandler* pViewHandler)
    : QObject()
    , m_pViewHandler(pViewHandler)
    , m_UseLodWhileMoving(true)
    , m_DefaultNavigationType(GLC_MoverController::TrackBall)
    , m_DoubleClicking(false)
    , m_UserState(0)
{
    Q_ASSERT(NULL != m_pViewHandler);
}

GLC_InputEventInterpreter::~GLC_InputEventInterpreter()
{

}

void GLC_InputEventInterpreter::setMover(GLC_MoverController::MoverType moverType, const GLC_UserInput &userInputs)
{
    m_pViewHandler->moverControllerHandle()->setActiveMover(moverType, userInputs);
    if (m_UseLodWhileMoving) {
        m_pViewHandler->world().collection()->setLodUsage(true, m_pViewHandler->viewportHandle());
    }
    m_pViewHandler->updateGL();
}

bool GLC_InputEventInterpreter::move(const GLC_UserInput &userInputs)
{
    bool subject= m_pViewHandler->moverControllerHandle()->move(userInputs);
    if (subject)
    {
        m_pViewHandler->clearSelectionBuffer();
        m_pViewHandler->updateGL();
    }

    return subject;
}

void GLC_InputEventInterpreter::setNoMover()
{
    m_pViewHandler->moverControllerHandle()->setNoMover();
    if (m_UseLodWhileMoving) {
        m_pViewHandler->world().collection()->setLodUsage(false, m_pViewHandler->viewportHandle());
    }
    m_pViewHandler->updateGL();
}

void GLC_InputEventInterpreter::select(int x, int y, GLC_SelectionEvent::Modes modes)
{
    QPair<GLC_SelectionSet, GLC_Point3d> selection= m_pViewHandler->selectAndUnproject(x, y, modes);
    GLC_SelectionEvent selectionEvent(modes, selection.first);
    m_pViewHandler->selectionUpdated(selectionEvent);
}
