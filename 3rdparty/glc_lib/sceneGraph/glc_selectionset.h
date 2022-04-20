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
//! \file glc_selectionset.h interface for the GLC_SelectionSet class.

#ifndef GLC_SELECTIONSET_H_
#define GLC_SELECTIONSET_H_

#include <QHash>
#include <QList>
#include <QSet>
#include <QMetaType>

#include "glc_structoccurrence.h"
#include "../glc_global.h"

#include "../glc_config.h"

class GLC_WorldHandle;
class GLC_World;

typedef QSet<GLC_uint> PrimitiveSelection;
typedef QHash<GLC_uint, PrimitiveSelection> BodySelection;
typedef QHash<GLC_uint, BodySelection> OccurrenceSelection;

//////////////////////////////////////////////////////////////////////
//! \class GLC_SelectionSet
/*! \brief GLC_SelectionSet : Occurrence id, Body id and primitive id selection set */
//////////////////////////////////////////////////////////////////////
class GLC_LIB_EXPORT GLC_SelectionSet
{
//////////////////////////////////////////////////////////////////////
/*! @name Constructor / Destructor */
//@{
//////////////////////////////////////////////////////////////////////

public:
    //! Construct an empty orphan selection set
    GLC_SelectionSet();

	//! Construct the selection set associated to the given GLC_WorldHandle
    explicit GLC_SelectionSet(GLC_WorldHandle* pWorld);

    //! Construct an empty selection set of the GLC_WorldHandle of the given GLC_World
    explicit GLC_SelectionSet(GLC_World& world);

    GLC_SelectionSet(const GLC_SelectionSet& other);


    virtual ~GLC_SelectionSet();

//@}

//////////////////////////////////////////////////////////////////////
/*! \name Get Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:

    //! Return true if this selection set is belong to same worldHandle od the given selection set
    bool belongsToSameWorldHandle(const GLC_SelectionSet& selectionSet) const;

	//! Return true if this selection set is empty
	bool isEmpty() const;

    //! Return the number of occurrence in this selection set
    inline int size() const
	{return count();}

    //! Return the number of occurrence in this selection set
	int count() const;

    //! Return the number of body in this selection set
    long bodyCount() const;

    //! Return the number of primitive of this selection set
    long primitiveCount() const;

    //! Return the occurrence selection hash table
    inline const OccurrenceSelection& occurrenceSelection() const
    {return m_OccurrenceSelection;}

    //! Return the list of selected View instance id
    QList<GLC_uint> idList() const;

    //! Return the first id of this selection set
    /*! If selection set is empty, return 0*/
    GLC_uint firstId() const;

    //! Return the list of selected occurrences
    QList<GLC_StructOccurrence*> occurrencesList() const;

    //! Return true if this selection set contains the given occurrence
    bool contains(const GLC_StructOccurrence* pOccurrence) const
    {return contains(pOccurrence->id());}

    //! Return true if this selection set contains the given occurrence id
    bool contains(GLC_uint occurrenceId) const
    {return m_OccurrenceSelection.contains(occurrenceId);}

    //! Return true if this selection contains the given body id of the given occurrence id
    bool contains(GLC_uint occId, GLC_uint bodyId);

    //! Return true if this selection contains the given primitive id of the given body id of the given occurrence id
    bool contains(GLC_uint occId, GLC_uint bodyId, GLC_uint primitiveId);

    //! Returns true if the other selection set is equal to this selection set set; otherwise returns false.
    bool operator==(const GLC_SelectionSet& other) const;

    //! Returns true if the other selection set is not equal to this selection set set; otherwise returns false.
    inline bool operator!=(const GLC_SelectionSet& other) const
    {return !(this->operator ==(other));}

    //! Return the list of selected bodies id of the given occurrence id
    QList<GLC_uint> selectedBodies(GLC_uint occurrenceId) const;

    //! Return the list of selected primitive of the given occurrence id and body id
    QList<GLC_uint> selectedPrimitive(GLC_uint occId, GLC_uint bodyId);

//@}
//////////////////////////////////////////////////////////////////////
/*! \name Set Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:
    //! Set the attached world to this selection set
    void setAttachedWorld(GLC_World world);

    //! Assigns the other selection set and return a reference to this selection set
    GLC_SelectionSet& operator=(const GLC_SelectionSet& other);

    //! Insert the given Occurrence into the selection set and return true on success
    /*! The given occurrence must belongs to this selection set's world*/
    bool insert(GLC_StructOccurrence* pOccurrence);

    //! Insert the given Occurrence id into the selection set and return true on success
    /*! The given occurrence id must belongs to this selection set's world*/
    bool insert(GLC_uint occurrenceId);

    //! Insert the given Body id of the given Occurrence id into the selection set and return true on success
    /*! The given occurrence id must belongs to this selection set's world*/
    bool insert(GLC_uint occurrenceId, GLC_uint bodyId);

    //! Insert he given Primitive Id of the given Body id of the given Occurrence id into the selection set and return true on success
    /*! The given occurrence id must belongs to this selection set's world*/
    bool insert(GLC_uint occurrenceId, GLC_uint bodyId, GLC_uint primitiveId);

    //! Remove the given occurrence from the selection set and return true on success
    /*! The given occurrence must belongs to this selection set's world*/
    bool remove(GLC_StructOccurrence* pOccurrence);

    //! Remove the given occurrence from the selection set and return true on success
    /*! The given occurrence id must belongs to this selection set's world*/
    bool remove(GLC_uint occurrenceId);

    //! Remove the given Body id of the given Occurrence id into the selection set and return true on success
    /*! The given occurrence must belongs to this selection set's world*/
    bool remove(GLC_uint occurrenceId, GLC_uint bodyId);

    //! Remove the given Primitive Id of the given Body id of the given Occurrence id into the selection set and return true on success
    /*! The given occurrence must belongs to this selection set's world*/
    bool remove(GLC_uint occurrenceId, GLC_uint bodyId, GLC_uint primitiveId);

	//! Clear this selection set
	void clear();

    //! if current wordHandle is not NULL, remove id that are not present in current worlHandle
    void clean();

    //! Each item in the other selection set that isn't already in this selectionset is inserted into this set.
    /*! A reference to this set is returned.*/
    GLC_SelectionSet& unite(const GLC_SelectionSet& other);

    //! Each item in the other selection set that isn't already in this selectionset is inserted into this set.
    /*! Each item present int this and other selection set are removed from  this selection set
     * A reference to this set is returned.*/
    GLC_SelectionSet& exclusiveUnite(const GLC_SelectionSet& other);

    //! Removes all items from this selection set that are contained in the other selection set. Returns a reference to this set.
    GLC_SelectionSet& substract(const GLC_SelectionSet& other);

//@}

//////////////////////////////////////////////////////////////////////
// Private members
//////////////////////////////////////////////////////////////////////
private:
	//! The worldHandle attached to this selection set
    GLC_WorldHandle* m_pWorldHandle;

    //! Selection record (Occurrence, Body and Primitive)
    OccurrenceSelection m_OccurrenceSelection;
};

Q_DECLARE_METATYPE(GLC_SelectionSet)

#endif /* GLC_SELECTIONSET_H_ */
