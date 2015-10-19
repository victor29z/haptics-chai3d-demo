//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \author    Dan Morris
    \author    Chris Sewell
    \version   3.0.0 $Rev: 831 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "world/CMesh.h"
//---------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionAABB.h"
#include "files/CFileModel3DS.h"
#include "files/CFileModelOBJ.h"
#include <algorithm>
#include <vector>
#include <list>
#include <utility>
#include <set>
#include <set>
//---------------------------------------------------------------------------
using std::vector;
using std::pair;
using std::set;
using std::multiset;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cMesh.

    \fn     cMesh::cMesh(cMaterial* a_material)

    \param  a_material  Material property to be applied to object.
*/
//===========================================================================
cMesh::cMesh(cMaterial* a_material)
{
    // create array of vertices
    m_vertices = new vector<cVertex>;

    // create list of free slots in the vertex array
    m_freeVertices = new list<unsigned int>;

    // create array of triangles
    m_triangles = new vector<cTriangle>;

    // create list of free slots in the triangle array
    m_freeTriangles = new list<unsigned int>;

    // create array of edges
    m_edges = new vector<cEdge>;

    // should normals be displayed?
    m_showNormals = false;

    // if normals are displayed, this value defines their length.
    m_normalsLength = 0.1;

    // if normals are displayed, this defines their color
    m_normalsColor.set(1.0, 0.0, 0.0);

    // should edges be displayed?
    m_showEdges = true;

    // Color used to render lines representing edges.
    m_edgeLineColor.setBlack();

    // width of edge lines
    m_edgeLineWidth = 1.0;

    // should the frame (X-Y-Z) be displayed?
    m_showFrame = false;

    // set default collision detector
    m_collisionDetector = NULL;

    // display lists disabled by default
    m_useDisplayList   = false;

    // vertex array disabled by default
    m_useVertexArrays = false;

    // set material properties
    if (a_material == NULL)
    {
        m_material = new cMaterial();
        m_material->setShininess(100);
        m_material->m_ambient.set ((float)0.3, (float)0.3, (float)0.3);
        m_material->m_diffuse.set ((float)0.1, (float)0.7, (float)0.8);
        m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);
    }
    else
    {
        m_material = a_material;
    }
}


//===========================================================================
/*!
    Destructor of cMesh.

    \fn     cMesh::~cMesh()
*/
//===========================================================================
cMesh::~cMesh()
{
    // clear vertex array
    m_vertices->clear();

    // clear list of free vertices
    m_freeVertices->clear();

    // clear triangle array
    m_triangles->clear();

    // clear list of free triangle.
    m_freeTriangles->clear();

    // Delete any allocated display lists
    m_displayList.invalidate();
    m_displayListEdges.invalidate();
}


//===========================================================================
/*!
    Create a copy of itself.

    \fn         cMesh* cMesh::copy(const bool a_duplicateMaterialData,
                   const bool a_duplicateTextureData, 
                   const bool a_duplicateMeshData,
                   const bool a_buildCollisionDetector)

    \param      a_duplicateMaterialData  If \b true, material (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateTextureData  If \b true, texture data (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateMeshData  If \b true, mesh data (if available) is duplicated, otherwise it is shared.
    \param      a_buildCollisionDetector  If \b true, collision detector (if available) is duplicated, otherwise it is shared.

	\return		Return new object.
*/
//===========================================================================
cMesh* cMesh::copy(const bool a_duplicateMaterialData,
                   const bool a_duplicateTextureData, 
                   const bool a_duplicateMeshData,
                   const bool a_buildCollisionDetector)
{
    // create new instance
    cMesh* obj = new cMesh(NULL);

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

    // copy mesh data
    if (a_duplicateMeshData)
    {
        // duplicate mesh data
        obj->m_vertices->reserve(m_vertices->size());
        {
            vector<cVertex>::iterator it = obj->m_vertices->begin();
		    for (it = m_vertices->begin(); it < m_vertices->end(); it++)
            {
                obj->m_vertices->push_back(*it);
            }
        }

        obj->m_triangles->reserve(m_triangles->size());
        {
            vector<cTriangle>::iterator it;
		    for (it = m_triangles->begin(); it < m_triangles->end(); it++)
            {
                obj->m_triangles->push_back(*it);
            }
        }

        if (a_buildCollisionDetector)
        {
            double radius = 0.0;
            if (m_collisionDetector)
            {
                radius = m_collisionDetector->getTriangleBoundaryRadius();
            }
            obj->createAABBCollisionDetector(radius);
        }
    }
    else
    {
        // share mesh data
        obj->m_vertices = m_vertices;
        obj->m_triangles = m_triangles;
        obj->m_edges = m_edges;
        if (a_buildCollisionDetector)
        {   
            if (m_collisionDetector)
            {
                obj->m_collisionDetector = m_collisionDetector;
            }
            else
            {
                obj->createAABBCollisionDetector(0.0);
            }
        }
    }

    // return
    return (obj);
}


//===========================================================================
/*!
	Compute the center of mass of this mesh, based on vertex positions.

	\fn		cVector3d cMesh::getCenterOfMass()

	\return	Return center of mass.		
*/
//===========================================================================
cVector3d cMesh::getCenterOfMass()
{
    cVector3d centerOfMass(0,0,0);
    unsigned long numVertices = (unsigned long)(m_vertices->size());

    if (numVertices != 0) 
    {
        for(unsigned int i=0; i<numVertices; i++)
        {
            cVector3d pos = m_vertices->at(i).getLocalPos();
            centerOfMass += pos;
        }
        centerOfMass /= ((double)(numVertices));
    }

    return (centerOfMass);
}


//===========================================================================
/*!
     This enables the use of vertex arrays for mesh rendering. This
     mode can be faster than the classical approach, however crashes
     sometime occur on certain types of graphic cards.

     In general, if you aren't having problems with rendering performance,
     don't bother with this.

     \fn       void cMesh::setUseVertexArrays(const bool a_useVertexArrays)

     \param    a_useVertexArrays  If \b true, this mesh will be rendered using vertex array technique.
*/
//===========================================================================
void cMesh::setUseVertexArrays(const bool a_useVertexArrays)
{
    // update changes to object
    m_useVertexArrays = a_useVertexArrays;
}


//===========================================================================
/*!
     Retrieve a pointer to a given triangle by passing its index number.

     \fn		cTriangle* cMesh::getTriangle(unsigned int a_index)

     \param		a_index  Index number of triangle.

	 \return	Return tointer to triangle.
*/
//===========================================================================
cTriangle* cMesh::getTriangle(unsigned int a_index)	
{ 
	if (a_index < m_triangles->size()) 
	{ 
		return (&(m_triangles->at(a_index))); 
	}
	else 
	{ 
		return (NULL); 
	}
}


//===========================================================================
/*!
     Return the number of stored triangles.

     \fn		unsigned int cMesh::getNumTriangles()

	 \return	Return the number of triangles.
*/
//===========================================================================
unsigned int cMesh::getNumTriangles()
{ 
	return (unsigned int)(m_triangles->size()); 
}


//===========================================================================
/*!
    Create a new vertex and add it to the vertex list.

    \fn         unsigned int cMesh::newVertex(const double a_x, 
										   	  const double a_y, 
											  const double a_z)
    \param      a_x  X coordinate of vertex.
    \param      a_y  Y coordinate of vertex.
    \param      a_z  Z coordinate of vertex.

    \return     Return index position in vertices list of new vertex.
*/
//===========================================================================
unsigned int cMesh::newVertex(const double a_x, const double a_y, const double a_z)
{
    unsigned int index;

    // check if there is any available vertex on the free list
    if (m_freeVertices->size() > 0)
    {
        index = m_freeVertices->front();
        m_freeVertices->erase(m_freeVertices->begin());
    }

    // No vertex is available on the free list so create a new one from the array
    else
    {
        // allocate new vertex
        index = (unsigned int)(m_vertices->size());
        cVertex newVertex(a_x, a_y, a_z);
        newVertex.m_index = index;
        m_vertices->push_back(newVertex);
    }

    // return the index at which I inserted this vertex in my vertex array
    return index;
}


//===========================================================================
/*!
    Create a new vertex and add it to the vertex list.

    \fn         unsigned int cMesh::newVertex(const double a_x, 
                              const double a_y, 
                              const double a_z,
                              const double a_normalX, 
                              const double a_normalY, 
                              const double a_normalZ)

    \param      a_x  X coordinate of vertex.
    \param      a_y  Y coordinate of vertex.
    \param      a_z  Z coordinate of vertex.
    \param      a_normalX  X coordinate of normal associated with vertex.
    \param      a_normalY  Y coordinate of normal associated with vertex.
    \param      a_normalZ  Z coordinate of normal associated with vertex.

    \return     Return index position in vertices list of new vertex.
*/
//===========================================================================
unsigned int cMesh::newVertex(const double a_x, 
                              const double a_y, 
                              const double a_z,
                              const double a_normalX, 
                              const double a_normalY, 
                              const double a_normalZ)
{
    unsigned int index = newVertex(a_x, a_y, a_z);
    m_vertices->at(index).m_normal.set(a_normalX, a_normalY, a_normalZ);
    return (index);
}


//===========================================================================
/*!
    Create a new vertex and add it to the vertex list.

    \fn         unsigned int cMesh::newVertex(const double a_x, 
							  const double a_y, 
							  const double a_z,
							  const double a_normalX, 
							  const double a_normalY, 
							  const double a_normalZ,
							  const double a_textureCoordX,
							  const double a_textureCoordY,
							  const double a_textureCoordZ)

    \param      a_x  X coordinate of vertex.
    \param      a_y  Y coordinate of vertex.
    \param      a_z  Z coordinate of vertex.
    \param      a_normalX  X coordinate of normal associated with vertex.
    \param      a_normalY  Y coordinate of normal associated with vertex.
    \param      a_normalZ  Z coordinate of normal associated with vertex.
    \param      a_textureCoordX  X component of texture coordinate.
    \param      a_textureCoordY  Y component of texture coordinate.
    \param      a_textureCoordZ  Z component of texture coordinate.

    \return     Return index position in vertices list of new vertex.
*/
//===========================================================================
unsigned int cMesh::newVertex(const double a_x, 
							  const double a_y, 
							  const double a_z,
							  const double a_normalX, 
							  const double a_normalY, 
							  const double a_normalZ,
							  const double a_textureCoordX,
							  const double a_textureCoordY,
							  const double a_textureCoordZ)
{
    unsigned int index = newVertex(a_x, a_y, a_z);
    m_vertices->at(index).m_normal.set(a_normalX, a_normalY, a_normalZ);
	m_vertices->at(index).m_texCoord.set(a_textureCoordX, a_textureCoordY, a_textureCoordZ);
    return (index);
}


//===========================================================================
/*!
    Create a new vertex and add it to the vertex list.

    \fn         unsigned int cMesh::newVertex(const cVector3d& a_pos)

    \param      a_pos  Position of new vertex.

    \return     Return index position in vertices list of new vertex.
*/
//===========================================================================
unsigned int cMesh::newVertex(const cVector3d& a_pos) 
{ 
    unsigned int index;

    // check if there is any available vertex on the free list
    if (m_freeVertices->size() > 0)
    {
        index = m_freeVertices->front();
        m_freeVertices->erase(m_freeVertices->begin());
    }

    // No vertex is available on the free list so create a new one from the array
    else
    {
        // allocate new vertex
        index = (unsigned int)(m_vertices->size());
        cVertex newVertex;
        newVertex.m_index = index;
        m_vertices->push_back(newVertex);
    }

    // set data
    cVertex* vertex = getVertex(index);
    vertex->setLocalPos(a_pos);

    // return vertex index
    return (index);
}


//===========================================================================
/*!
    Create a new vertex and add it to the vertex list.

    \fn         unsigned int cMesh::newVertex(const cVector3d& a_pos, 
							  const cVector3d& a_normal) 

    \param      a_pos  Position of new vertex
    \param      a_normal  Normal vector associated with new vertex

    \return     Return index position in vertices list of new vertex.
*/
//===========================================================================
unsigned int cMesh::newVertex(const cVector3d& a_pos, 
							  const cVector3d& a_normal) 
{ 
    unsigned int index;

    // check if there is any available vertex on the free list
    if (m_freeVertices->size() > 0)
    {
        index = m_freeVertices->front();
        m_freeVertices->erase(m_freeVertices->begin());
    }

    // No vertex is available on the free list so create a new one from the array
    else
    {
        // allocate new vertex
        index = (unsigned int)(m_vertices->size());
        cVertex newVertex;
        newVertex.m_index = index;
        m_vertices->push_back(newVertex);
    }

    // set data
    cVertex* vertex = getVertex(index);
    vertex->setLocalPos(a_pos);
    vertex->setNormal(a_normal);

    // return vertex index
    return (index);
}


//===========================================================================
/*!
    Create a new vertex and add it to the vertex list.

    \fn         unsigned int cMesh::newVertex(const cVector3d& a_pos, 
							  const cVector3d& a_normal,
							  const cVector3d& a_textureCoord)

    \param      a_pos  Position of new vertex.
    \param      a_normal  Normal vector associated with new vertex
	\param		a_textureCoord  Texture coordinate.

    \return     Return index position in vertices list of new vertex.
*/
//===========================================================================
unsigned int cMesh::newVertex(const cVector3d& a_pos, 
							  const cVector3d& a_normal,
							  const cVector3d& a_textureCoord)
{
    unsigned int index;

    // check if there is any available vertex on the free list
    if (m_freeVertices->size() > 0)
    {
        index = m_freeVertices->front();
        m_freeVertices->erase(m_freeVertices->begin());
    }

    // No vertex is available on the free list so create a new one from the array
    else
    {
        // allocate new vertex
        index = (unsigned int)(m_vertices->size());
        cVertex newVertex;
        newVertex.m_index = index;
        m_vertices->push_back(newVertex);
    }

    // set data
    cVertex* vertex = getVertex(index);
    vertex->setLocalPos(a_pos);
    vertex->setNormal(a_normal);
    vertex->setTexCoord(a_textureCoord);

    // return vertex index
    return (index);
}


//===========================================================================
/*!
    Create a new vertex and add it to the vertex list.

    \fn         unsigned int cMesh::newVertex(const cVector3d& a_pos, 
                              const cVector3d& a_normal,
						      const cVector3d& a_textureCoord,
                              const cColorf& a_color)

    \param      a_pos  Position of new vertex.
    \param      a_normal  Normal vector associated with new vertex
	\param		a_textureCoord  Texture coordinate.
    \param      a_color  Color.

    \return     Return index position in vertices list of new vertex.
*/
//===========================================================================
unsigned int cMesh::newVertex(const cVector3d& a_pos, 
                              const cVector3d& a_normal,
						      const cVector3d& a_textureCoord,
                              const cColorf& a_color)
{
    unsigned int index;

    // check if there is any available vertex on the free list
    if (m_freeVertices->size() > 0)
    {
        index = m_freeVertices->front();
        m_freeVertices->erase(m_freeVertices->begin());
    }

    // No vertex is available on the free list so create a new one from the array
    else
    {
        // allocate new vertex
        index = (unsigned int)(m_vertices->size());
        cVertex newVertex;
        newVertex.m_index = index;
        m_vertices->push_back(newVertex);
    }

    // set data
    cVertex* vertex = getVertex(index);
    vertex->setLocalPos(a_pos);
    vertex->setNormal(a_normal);
    vertex->setTexCoord(a_textureCoord);
    vertex->setColor(a_color);

    // return vertex index
    return (index);
}


//===========================================================================
/*!
    Create a new vertex for each supplied position and add it to the vertex list.

    \fn         void cMesh::addVertices(const cVector3d* a_vertexPositions,
						const unsigned int& a_numVertices)

    \param      a_vertexPositions List of vertex positions to add
    \param      a_numVertices Number of vertices in a_vertexPositions
*/
//===========================================================================
void cMesh::addVertices(const cVector3d* a_vertexPositions,
						const unsigned int& a_numVertices)
{
    const cVector3d* end = a_vertexPositions + a_numVertices;
    while(a_vertexPositions != end) 
    {
        newVertex(*a_vertexPositions);
        a_vertexPositions++;
    }
}


//===========================================================================
/*!
    Remove the vertex at the specified position in my vertex array. A vertex
	can only be removed if it is longer part of a triangle composing the mesh.

    \fn       bool cMesh::removeVertex(const unsigned int a_index)
    \param    a_index  Index number of vertex.

    \return   Return \b true if operation succeeded.
*/
//===========================================================================
bool cMesh::removeVertex(const unsigned int a_index)
{
    // get vertex to be removed
    cVertex* vertex = &m_vertices->at(a_index);

	// verify that current vertex is not currently used by any triangles
	if (vertex->m_nTriangles > 0) { return (false); }

    // check if vertex has not already been removed
    if (vertex->m_allocated == false) { return (false); }

    // deactivate vertex
    vertex->m_allocated = false;

	// reset position of vertex
	vertex->setLocalPos(0.0, 0.0, 0.0);

    // add vertex to free list
    m_freeVertices->push_back(a_index);

	// mark mesh for update
	invalidateDisplayList();

    // return success
    return (true);
}


//===========================================================================
/*!
    Create a new triangle by passing vertex indices

    \fn         unsigned int cMesh::newTriangle(const unsigned int a_indexVertex0, 
                                const unsigned int a_indexVertex1,
                                const unsigned int a_indexVertex2)

    \param      a_indexVertex0   index position of vertex 0.
    \param      a_indexVertex1   index position of vertex 1.
    \param      a_indexVertex2   index position of vertex 2.

    \return     Return the index of the new triangle in my triangle array.
*/
//===========================================================================
unsigned int cMesh::newTriangle(const unsigned int a_indexVertex0, 
                                const unsigned int a_indexVertex1,
                                const unsigned int a_indexVertex2)
{
    unsigned int index;

    // check if there is an available slot on the free triangle list
    if (m_freeTriangles->size() > 0)
    {
        index = m_freeTriangles->front();
        m_triangles->at(index).m_allocated = true;
        m_freeTriangles->pop_front();
        m_triangles->at(index).setVertices(a_indexVertex0, a_indexVertex1, a_indexVertex2);
        m_triangles->at(index).setOwner(this);
    }

    // no triangle is available from the free list so create a new one from the array
    else
    {
        // allocate new triangle
        index = (unsigned int)(m_triangles->size());
        cTriangle newTriangle(this, a_indexVertex0, a_indexVertex1, a_indexVertex2);
        newTriangle.m_index = index;
        newTriangle.m_allocated = true;
        m_triangles->push_back(newTriangle);
    }

    m_vertices->at(a_indexVertex0).m_allocated = true;
    m_vertices->at(a_indexVertex0).m_nTriangles++;
    m_vertices->at(a_indexVertex1).m_allocated = true;
    m_vertices->at(a_indexVertex1).m_nTriangles++;
    m_vertices->at(a_indexVertex2).m_allocated = true;
    m_vertices->at(a_indexVertex2).m_nTriangles++;

	// mark mesh for update
	invalidateDisplayList();

    // return the index at which I inserted this triangle in my triangle array
    return (index);
}

   
//===========================================================================
/*!
     Create a new triangle and three new vertices by passing vertex positions

     \fn       unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
                                const cVector3d& a_vertex1,
                                const cVector3d& a_vertex2)

     \param    a_vertex0   Position of vertex 0.
     \param    a_vertex1   Position of vertex 1.
     \param    a_vertex2   Position of vertex 2.

     \return   Return index position of new triangle.
*/
//===========================================================================
unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
                                const cVector3d& a_vertex1,
                                const cVector3d& a_vertex2)
{
    // create three new vertices
    unsigned int indexVertex0 = newVertex(a_vertex0);
    unsigned int indexVertex1 = newVertex(a_vertex1);
    unsigned int indexVertex2 = newVertex(a_vertex2);

    // create new triangle
    unsigned int indexTriangle = newTriangle(indexVertex0, indexVertex1, indexVertex2);

	// mark mesh for update
	invalidateDisplayList();

    // return index of new triangle.
    return (indexTriangle);
}


//===========================================================================
/*!
    Create a new triangle and three new vertices by passing vertex indices
    and normals.

    \fn         unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
                                const cVector3d& a_vertex1,
                                const cVector3d& a_vertex2,
                                const cVector3d& a_normal0, 
                                const cVector3d& a_normal1,
                                const cVector3d& a_normal2)

    \param      a_indexVertex0   Index number of vertex 0.
    \param      a_indexVertex1   Index number of vertex 1.
    \param      a_indexVertex2   Index number of vertex 2.
    \param      a_normal0   Normal of vertex 0.
    \param      a_normal1   Normal position of vertex 1.
    \param      a_normal2   Normal position of vertex 2.

    \return     Return the index of the new triangle in my triangle array.
*/
//===========================================================================
unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
                                const cVector3d& a_vertex1,
                                const cVector3d& a_vertex2,
                                const cVector3d& a_normal0, 
                                const cVector3d& a_normal1,
                                const cVector3d& a_normal2)
{
    unsigned int index = newTriangle(a_vertex0, a_vertex1, a_vertex2);
    m_triangles->at(index).getVertex0()->setNormal(a_normal0);
    m_triangles->at(index).getVertex1()->setNormal(a_normal1);
    m_triangles->at(index).getVertex2()->setNormal(a_normal2);

	// mark mesh for update
	invalidateDisplayList();

    // return result
    return (index);
}


//===========================================================================
/*!
	 Create a new triangle and three new vertices by passing vertex 
	 positions, normals and texture coordinates.

	 \fn       unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
								const cVector3d& a_vertex1,
								const cVector3d& a_vertex2,
								const cVector3d& a_normal0, 
								const cVector3d& a_normal1,
								const cVector3d& a_normal2,
								const cVector3d& a_textureCoord0, 
								const cVector3d& a_textureCoord1,
								const cVector3d& a_textureCoord2)

	\param		a_vertex0   Position of vertex 0.
	\param		a_vertex1   Position of vertex 1.
	\param		a_vertex2   Position of vertex 2.
	\param		a_normal0   Normal of vertex 0.
	\param		a_normal1   Normal position of vertex 1.
	\param		a_normal2   Normal position of vertex 2.
	\param		a_textureCoord0   Texture coordinate of vertex 0.
	\param		a_textureCoord1   Texture coordinate of vertex 1.
	\param		a_textureCoord2   Texture coordinate of vertex 2.

	\return		Return index position of new triangle.
*/
//===========================================================================
unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
								const cVector3d& a_vertex1,
								const cVector3d& a_vertex2,
								const cVector3d& a_normal0, 
								const cVector3d& a_normal1,
								const cVector3d& a_normal2,
								const cVector3d& a_textureCoord0, 
								const cVector3d& a_textureCoord1,
								const cVector3d& a_textureCoord2)
{
    cVertex* vertex;

	// create new triangle
	unsigned int index = newTriangle(a_vertex0, a_vertex1, a_vertex2);
    
	// set attributes
	vertex = m_triangles->at(index).getVertex0();
	vertex->setNormal(a_normal0);
	vertex->setTexCoord(a_textureCoord0);

	vertex = m_triangles->at(index).getVertex1();
	vertex->setNormal(a_normal1);
	vertex->setTexCoord(a_textureCoord1);

	vertex = m_triangles->at(index).getVertex2();
	vertex->setNormal(a_normal2);
	vertex->setTexCoord(a_textureCoord2);

	// mark mesh for update
	invalidateDisplayList();

    // return result
    return (index);
}


//===========================================================================
/*!
	 Create a new triangle and three new vertices by passing vertex 
	 positions, normals and texture coordinates.

	 \fn       unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
								const cVector3d& a_vertex1,
								const cVector3d& a_vertex2,
								const cVector3d& a_normal0, 
								const cVector3d& a_normal1,
								const cVector3d& a_normal2,
								const cVector3d& a_textureCoord0, 
								const cVector3d& a_textureCoord1,
								const cVector3d& a_textureCoord2,
                                const cColorf& a_colorVertex0,
                                const cColorf& a_colorVertex1,
                                const cColorf& a_colorVertex2)

	\param		a_vertex0   Position of vertex 0.
	\param		a_vertex1   Position of vertex 1.
	\param		a_vertex2   Position of vertex 2.
	\param		a_normal0   Normal of vertex 0.
	\param		a_normal1   Normal position of vertex 1.
	\param		a_normal2   Normal position of vertex 2.
	\param		a_textureCoord0   Texture coordinate of vertex 0.
	\param		a_textureCoord1   Texture coordinate of vertex 1.
	\param		a_textureCoord2   Texture coordinate of vertex 2.
    \param      a_colorVertex0  Color at vertex 0.
    \param      a_colorVertex1  Color at vertex 1.
    \param      a_colorVertex2  Color at vertex 2.

	\return		Return index position of new triangle.
*/
//===========================================================================
unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
								const cVector3d& a_vertex1,
								const cVector3d& a_vertex2,
								const cVector3d& a_normal0, 
								const cVector3d& a_normal1,
								const cVector3d& a_normal2,
								const cVector3d& a_textureCoord0, 
								const cVector3d& a_textureCoord1,
								const cVector3d& a_textureCoord2,
                                const cColorf& a_colorVertex0,
                                const cColorf& a_colorVertex1,
                                const cColorf& a_colorVertex2)
{
    cVertex* vertex;

	// create new triangle
	unsigned int index = newTriangle(a_vertex0, a_vertex1, a_vertex2);
    
	// set attributes
	vertex = m_triangles->at(index).getVertex0();
	vertex->setNormal(a_normal0);
	vertex->setTexCoord(a_textureCoord0);
    vertex->setColor(a_colorVertex0);

	vertex = m_triangles->at(index).getVertex1();
	vertex->setNormal(a_normal1);
	vertex->setTexCoord(a_textureCoord1);
    vertex->setColor(a_colorVertex1);

	vertex = m_triangles->at(index).getVertex2();
	vertex->setNormal(a_normal2);
	vertex->setTexCoord(a_textureCoord2);
    vertex->setColor(a_colorVertex2);

	// mark mesh for update
	invalidateDisplayList();

    // return result
    return (index);
}


//===========================================================================
/*!
     Remove a vertex from the vertex array by passing its index number.

     \fn       bool cMesh::removeTriangle(const unsigned int a_index)

     \param    a_index  Index number of vertex.

     \return   Return \b true if operation succeeded.
*/
//===========================================================================
bool cMesh::removeTriangle(const unsigned int a_index)
{
    // get triangle to be removed
    cTriangle* triangle = &m_triangles->at(a_index);

    // check if it has not already been removed
    if (triangle->m_allocated == false) { return (false); }

    // deactivate triangle
    triangle->m_allocated = false;

	// reduce triangle counter for each vertex
    m_vertices->at(triangle->m_indexVertex0).m_nTriangles--;
    m_vertices->at(triangle->m_indexVertex1).m_nTriangles--;
    m_vertices->at(triangle->m_indexVertex2).m_nTriangles--;

	// reset index to vertices
	triangle->m_indexVertex0 = 0;
	triangle->m_indexVertex1 = 0;
	triangle->m_indexVertex2 = 0;

    // add triangle to free list
    m_freeTriangles->push_back(a_index);

	// mark mesh for update
	invalidateDisplayList();

    // return success
    return (true);
}


//===========================================================================
/*!
     Create a list of edges by providing a threshold angle in degrees. All
     triangles for which the angle between their respective surface normals 
     are greater than the select angle threshold are added to the list of 
     edges.

     \fn        void cMesh::computeAllEdges(double a_angleThresholdDEG)

     \param     a_angleThresholdDEG  Threshold angle in degrees.
*/
//===========================================================================
void cMesh::computeAllEdges(double a_angleThresholdDEG)
{
    // clear current list of edges.
    clearAllEdges();

    // initlize variables
    int numtriangles = getNumTriangles();
    int numVertices  = getNumVertices();
    
    multiset<cEdge> edges;
    edges.clear();
    cEdge edge;
    multiset<cEdge>::iterator it;

    // setup angle threshold
    double ANGLE_THRESHOLD = cDegToRad(a_angleThresholdDEG);

    // process all triangles
    for (int i=0; i<numtriangles; i++)
    {
        cTriangle* t0 = getTriangle(i);
        int v0 = t0->getIndexVertex0();
        int v1 = t0->getIndexVertex1();
        int v2 = t0->getIndexVertex2();

        cVector3d z0 = getVertex(v0)->getLocalPos();
        cVector3d z1 = getVertex(v1)->getLocalPos();
        cVector3d z2 = getVertex(v2)->getLocalPos();

        cVector3d n0 = cComputeSurfaceNormal(z0, z1, z2);

        if (n0.length() > 0)
        {
            //////////////////////////////////////////////////////////////////
            // store reference to triangle
            //////////////////////////////////////////////////////////////////
            edge.m_triangle = i;
            
            //////////////////////////////////////////////////////////////////
            // edge 01 of triangle.
            //////////////////////////////////////////////////////////////////
            edge.set(getVertex(v0), getVertex(v1));
            it = edges.find(edge);
            if (it!=edges.end())
            {
                cTriangle* t1 = getTriangle((*it).m_triangle);
                cVector3d n1 = cComputeSurfaceNormal(t1->getVertex0()->getLocalPos(),
                                                     t1->getVertex1()->getLocalPos(),
                                                     t1->getVertex2()->getLocalPos());

                if (n1.length() > 0.0)
                {
                    if (cAngle(n0, n1) >= ANGLE_THRESHOLD)
                    {
                        m_edges->push_back(edge);
                    }
                }

				// remove edge as we have already found it dual
				edges.erase(it);
            }
            else
            {
                edges.insert(edge);
            }

 
            //////////////////////////////////////////////////////////////////
            // edge 02 of triangle.
            //////////////////////////////////////////////////////////////////
            edge.set(getVertex(v0), getVertex(v2));
            it = edges.find(edge);
            if (it!=edges.end())
            {
                cTriangle* t1 = getTriangle((*it).m_triangle);
                cVector3d n1 = cComputeSurfaceNormal(t1->getVertex0()->getLocalPos(),
                                                     t1->getVertex1()->getLocalPos(),
                                                     t1->getVertex2()->getLocalPos());

                if (n1.length() > 0.0)
                {
                    if (cAngle(n0, n1) >= ANGLE_THRESHOLD)
                    {
                        m_edges->push_back(edge);
                    }
                }

				// remove edge as we have already found it dual
				edges.erase(it);
            }
            else
            {
                edges.insert(edge);
            }

            //////////////////////////////////////////////////////////////////
            // edge 12 of triangle.
            //////////////////////////////////////////////////////////////////
            edge.set(getVertex(v1), getVertex(v2));
            it = edges.find(edge);
            if (it!=edges.end())
            {
                cTriangle* t1 = getTriangle((*it).m_triangle);
                cVector3d n1 = cComputeSurfaceNormal(t1->getVertex0()->getLocalPos(),
                                                     t1->getVertex1()->getLocalPos(),
                                                     t1->getVertex2()->getLocalPos());

                if (n1.length() > 0.0)
                {
                    if (cAngle(n0, n1) >= ANGLE_THRESHOLD)
                    {
                        m_edges->push_back(edge);
                    }
                }

				// remove edge as we have already found it dual
				edges.erase(it);
            }
            else
            {
                edges.insert(edge);
            }
        }
    }

	// store all edges whith no dual in the edge list
	multiset<cEdge>::iterator it2;
	for (it2 = edges.begin(); it2 != edges.end(); it2++)
	{
		m_edges->push_back(*it2);
	}
}


//===========================================================================
/*!
     Clear all edges.

     \fn       void cMesh::clearAllEdges()
*/
//===========================================================================
void cMesh::clearAllEdges()
{
    // clear all edges
    m_edges->clear();
    m_displayListEdges.invalidate();
}


//===========================================================================
/*!
     Clear all triangles and vertices.

     \fn       void cMesh::clear()
*/
//===========================================================================
void cMesh::clear()
{
    // clear all triangles
    m_triangles->clear();

    // clear all vertices
    m_vertices->clear();

    // clear all edges
    m_edges->clear();

    // clear free triangles list
    m_freeTriangles->clear();

    // clear free vertices list
    m_freeVertices->clear();
}


//===========================================================================
/*!
     Compute surface normals for every vertex in the mesh, by averaging
     the face normals of the triangle that include each vertex.

     \fn       void cMesh::computeAllNormals()
*/
//===========================================================================
void cMesh::computeAllNormals()
{
	// read number of vertices and triangles of object
	unsigned int numTriangles = (unsigned int)(m_triangles->size());
	unsigned int numVertices = (unsigned int)(m_vertices->size());

	// initialize all normals to zero
	for (unsigned int i=0; i<numVertices; i++)
	{
		m_vertices->at(i).setNormal(0.0, 0.0, 0.0);
	}

	// compute the normal of each triangle, add contribution to each vertex
	for (unsigned int i=0; i<numTriangles; i++)
	{
		cTriangle* nextTriangle = getTriangle(i);
		cVector3d vertex0 = nextTriangle->getVertex0()->getLocalPos();
        cVector3d vertex1 = nextTriangle->getVertex1()->getLocalPos();
        cVector3d vertex2 = nextTriangle->getVertex2()->getLocalPos();

		// compute normal vector
        cVector3d normal, v01, v02;
        vertex1.subr(vertex0, v01);
        vertex2.subr(vertex0, v02);
        v01.crossr(v02, normal);
        double length = normal.length();
		if (length > 0.0)
		{
			normal.div(length);
			nextTriangle->getVertex0()->m_normal.add(normal);
			nextTriangle->getVertex1()->m_normal.add(normal);
			nextTriangle->getVertex2()->m_normal.add(normal);
		}
	}

	// normalize all triangles
	for (unsigned int i=0; i<numVertices; i++)
	{
		if (m_vertices->at(i).getNormal().length() < 0.000000001)
		{
			bool error = true;
		}
		m_vertices->at(i).m_normal.normalize();
	}
}


//===========================================================================
/*!
     Compute the global position of all vertices

     \fn       void cMesh::updateGlobalPositions(const bool a_frameOnly)

     \param    a_frameOnly  If \b false, the global position of all vertices.
               is computed, otherwise this function does nothing.
*/
//===========================================================================
void cMesh::updateGlobalPositions(const bool a_frameOnly)
{
    if (a_frameOnly) return;

    unsigned int i,numVertices;
    numVertices = (unsigned int)(m_vertices->size());
    for (i=0; i<numVertices; i++)
    {
        m_vertices->at(i).computeGlobalPosition(m_globalPos,m_globalRot);
    }
}


//===========================================================================
/*!
     Invalidate any existing display lists.  You should call this on if you're using
     display lists and you modify mesh options, vertex positions, etc.

     \fn        void cMesh::invalidateDisplayList(const bool a_affectChildren)

     \param     a_affectChildren  If \b true, then children are updated too.
*/
//===========================================================================
void cMesh::invalidateDisplayList(const bool a_affectChildren)
{
    cGenericObject::invalidateDisplayList(a_affectChildren);

    // invalidate display list
    m_displayListEdges.invalidate();
}


//===========================================================================
/*!
     Set the alpha value at each vertex, in all of my material colors,
     optionally propagating the operation to my children.

     \fn	    void cMesh::setTransparencyLevel(const float a_level,
                                 const bool a_applyToTextures,
                                 const bool a_affectChildren)

     \param     a_level  Level of transparency ranging from 0.0 to 1.0.
     \param     a_applyToTextures  If \b true, then apply changes to texture.
     \param     a_affectChildren  If \b true, then children are updated too.
*/
//===========================================================================
void cMesh::setTransparencyLevel(const float a_level,
                                 const bool a_applyToTextures,
                                 const bool a_affectChildren)
{
    cGenericObject::setTransparencyLevel(a_level, a_applyToTextures, a_affectChildren);
                                        
    // apply the new value to all vertex colors
    unsigned int i, numItems;
    numItems = (unsigned int)(m_vertices->size());
    for(i=0; i<numItems; i++)
    {
        m_vertices->at(i).m_color.setA(a_level);
    }

    // invalidate display list
    m_displayListEdges.invalidate();
}


//===========================================================================
/*!
     Set color of each vertex.

     \fn        void cMesh::setVertexColor(const cColorf& a_color)

     \param     a_color   New color to be applied to each vertex.
*/
//===========================================================================
void cMesh::setVertexColor(const cColorf& a_color)
{
    // apply color to all vertex colors
    unsigned int i, numItems;
    numItems = (unsigned int)(m_vertices->size());
    for(i=0; i<numItems; i++)
    {
        m_vertices->at(i).m_color = a_color;
    }
}



//===========================================================================
/*!
    Shifts all vertex positions by the specified amount.

    \fn			void cMesh::offsetVertices(const cVector3d& a_offset,
						   const bool a_updateCollisionDetector)

    \param		a_offset  Translation to apply to each vertex.
	\param		a_updateCollisionDetector  If \b true, then update collision detector.
*/
//===========================================================================
void cMesh::offsetVertices(const cVector3d& a_offset,
						   const bool a_updateCollisionDetector)
{
    // offset all vertices
    int vertexcount = (int)(m_vertices->size());
    for(int i=0; i<vertexcount; i++)
    {
        m_vertices->at(i).m_localPos.add(a_offset);
    }

	// update boundary box
    m_boundaryBoxMin+=a_offset;
    m_boundaryBoxMax+=a_offset;

	// update collision detector if requested
    if (a_updateCollisionDetector && m_collisionDetector)
	{
        m_collisionDetector->initialize();
	}
}


//===========================================================================
/*!
    Reverse the normal for every vertex on this model.  Useful for models
    that started with inverted faces and thus gave inward-pointing normals.

    \fn        void cMesh::reverseAllNormals()
*/
//===========================================================================
void cMesh::reverseAllNormals()
{
	// reverse normals for this object
	if (m_vertices->size() > 0)
	{
		int numVertices = (int)(m_vertices->size());
		for(int i=0; i<numVertices; i++)
		{
			m_vertices->at(i).m_normal.mul(-1.0);
		}
	}
}


//===========================================================================
/*!
     Define the way normals are graphically rendered.

     \fn        void cMesh::setNormalsProperties(const double a_length, 
								 const cColorf& a_color)

     \param     a_length  Length of normals
     \param     a_color  Color of normals
*/
//===========================================================================
void cMesh::setNormalsProperties(const double a_length, 
								 const cColorf& a_color)
{
    m_normalsLength = cClamp0(a_length);
    m_normalsColor = a_color;
}


//===========================================================================
/*!
     Set graphic properties for edge-rendering.

     \fn        void cMesh::setEdgeProperties(const double a_lineWidth, 
                              const cColorf& a_lineColor)

     \param     a_lineWidth  Width of edge lines.
     \param     a_lineColor  Color of edge lines.
*/
//===========================================================================
void cMesh::setEdgeProperties(const double a_lineWidth, 
                              const cColorf& a_lineColor)
{
    m_edgeLineWidth = cMin(a_lineWidth, 1.0);
    m_edgeLineColor = a_lineColor;
    m_displayListEdges.invalidate();
}


//===========================================================================
/*!
     Compute the axis-aligned boundary box that encloses all triangles 
	 in this mesh.

     \fn       void cMesh::updateBoundaryBox()
*/
//===========================================================================
void cMesh::updateBoundaryBox()
{
    if (m_triangles->size() == 0)
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        return;
    }

    double xMin = C_LARGE;
    double yMin = C_LARGE;
    double zMin = C_LARGE;
    double xMax = -C_LARGE;
    double yMax = -C_LARGE;
    double zMax = -C_LARGE;;

    // loop over all my triangles
    for(unsigned int i=0; i<m_triangles->size(); i++)
    {
        // get next triangle
        cTriangle* nextTriangle = &m_triangles->at(i);

        if (nextTriangle->m_allocated)
        {
            cVector3d tVertex0 = m_vertices->at(nextTriangle->m_indexVertex0).m_localPos;
            xMin = cMin(tVertex0(0) , xMin);
            yMin = cMin(tVertex0(1) , yMin);
            zMin = cMin(tVertex0(2) , zMin);
            xMax = cMax(tVertex0(0) , xMax);
            yMax = cMax(tVertex0(1) , yMax);
            zMax = cMax(tVertex0(2) , zMax);

            cVector3d tVertex1 = m_vertices->at(nextTriangle->m_indexVertex1).m_localPos;
            xMin = cMin(tVertex1(0) , xMin);
            yMin = cMin(tVertex1(1) , yMin);
            zMin = cMin(tVertex1(2) , zMin);
            xMax = cMax(tVertex1(0) , xMax);
            yMax = cMax(tVertex1(1) , yMax);
            zMax = cMax(tVertex1(2) , zMax);

            cVector3d tVertex2 = m_vertices->at(nextTriangle->m_indexVertex2).m_localPos;
            xMin = cMin(tVertex2(0) , xMin);
            yMin = cMin(tVertex2(1) , yMin);
            zMin = cMin(tVertex2(2) , zMin);
            xMax = cMax(tVertex2(0) , xMax);
            yMax = cMax(tVertex2(1) , yMax);
            zMax = cMax(tVertex2(2) , zMax);
        }
    }

    if (m_triangles->size() > 0)
    {
        m_boundaryBoxMin.set(xMin, yMin, zMin);
        m_boundaryBoxMax.set(xMax, yMax, zMax);
    }
    else
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
    }
}


//===========================================================================
/*!
    Scale mesh with a uniform scale factor.

    \fn       void cMesh::scaleObject(const double& a_scaleFactor)

    \param    a_scaleFactor  Scale factor.
*/
//===========================================================================
void cMesh::scaleObject(const double& a_scaleFactor)
{
    unsigned int i, numItems;
    numItems = (unsigned int)(m_vertices->size());

    for(i=0; i<numItems; i++)
    {
        m_vertices->at(i).m_localPos.mul(a_scaleFactor);
    }

    m_boundaryBoxMax.mul(a_scaleFactor);
    m_boundaryBoxMin.mul(a_scaleFactor);
}


//===========================================================================
/*!
     Set up a Brute Force collision detector for this mesh.

     \fn       void cMesh::createBruteForceCollisionDetector()
*/
//===========================================================================
void cMesh::createBruteForceCollisionDetector()
{
    // delete previous collision detector
    if (m_collisionDetector != NULL)
    {
        delete m_collisionDetector;
        m_collisionDetector = NULL;
    }

    // create brute collision detector
    cCollisionBrute* col = new cCollisionBrute(m_triangles);
    col->initialize();

	// assign new collision detector
    m_collisionDetector = col;
}


//===========================================================================
/*!
     Set up an AABB collision detector for this mesh.

     \fn       void cMesh::createAABBCollisionDetector(const double a_radius)

	 \param	   a_radius  Bounding radius.
*/
//===========================================================================
void cMesh::createAABBCollisionDetector(const double a_radius)
{
    // delete previous collision detector
    if (m_collisionDetector != NULL)
    {
        delete m_collisionDetector;
        m_collisionDetector = NULL;
    }

    // create AABB collision detector
    cCollisionAABB* col = new cCollisionAABB(m_triangles);
    col->initialize(a_radius);

	// assign new collision detector
    m_collisionDetector = col;
}


//===========================================================================
/*!
	For mesh objects, this information is computed by the virtual tool
	when computing the finger-proxy model. More information can be found in 
	file cToolCursor.cpp undr methof computeInteractionForces()
	Both variables m_interactionProjectedPoint and m_interactionInside are
	assigned values based on the objects encountered by the proxy.

    \fn     void cMesh::computeLocalInteraction(const cVector3d& a_toolPos,
                                                const cVector3d& a_toolVel,
                                                const unsigned int a_IDN)

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void cMesh::computeLocalInteraction(const cVector3d& a_toolPos,
                                    const cVector3d& a_toolVel,
                                    const unsigned int a_IDN)
{
    // m_interactionProjectedPoint
    // m_interactionInside
}


//===========================================================================
/*!
     Render this mesh in OpenGL.  This method actually just prepares some
     OpenGL state, and uses renderMesh to actually do the rendering.

     \fn       void cMesh::render(cRenderOptions& a_options)
     \param    a_options  Rendering options
*/
//===========================================================================
void cMesh::render(cRenderOptions& a_options)
{
	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        // render normals
		// (note that we will not render the normals if we are currently creating
		// a shadow map). 
        if (m_showNormals && !a_options.m_creating_shadow_map) 
		{
			renderNormals();
		}

        // render edges
        if (m_showEdges && !a_options.m_creating_shadow_map)
		{
			renderEdges();
		}
    }

 	/////////////////////////////////////////////////////////////////////////
	// Render parts that use material properties
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency))
	{
        renderMesh(a_options);
	}
}


//===========================================================================
/*!
     Render a graphic representation of each normal of the mesh.

     \fn       void cMesh::renderNormals()
*/
//===========================================================================
void cMesh::renderNormals()
{
    // check if any normals to render
    if ((m_vertices->size() == 0))
    {
        return;
    }

    // disable lighting
    glDisable(GL_LIGHTING);

    // set line width
    glLineWidth(1.0);

    // set color
    glColor4fv( (const float *)&m_normalsColor);

    // render normals
	unsigned int nvertices = (unsigned int)(m_vertices->size());
	glBegin(GL_LINES);
	for(unsigned int i=0; i<nvertices; i++) 
	{
		if (m_vertices->at(i).m_allocated)
		{
			cVector3d v = m_vertices->at(i).m_localPos;
			cVector3d n = m_vertices->at(i).m_normal;
			glVertex3d(v(0) ,v(1) ,v(2) );

			n.mul(m_normalsLength);
			n.add(v);
			glVertex3d(n(0) ,n(1) ,n(2) );
		}
	}
	glEnd();

    // enable lighting
    glEnable(GL_LIGHTING);
}


//===========================================================================
/*!
     Render all edges.

     \fn       void cMesh::renderEdges()
*/
//===========================================================================
void cMesh::renderEdges()
{
    if (m_edges->size() == 0) { return; }

    /////////////////////////////////////////////////////////////////////////
    // SET PROPERTIES
    /////////////////////////////////////////////////////////////////////////

    // turn-off lighting
    glDisable(GL_LIGHTING);
    
    // setup color for rendering line edges
    m_edgeLineColor.render();

    // width of line edges
    glLineWidth(m_edgeLineWidth);

    // render all edges as triangles so that we can perform polygon offset
    glEnable(GL_POLYGON_OFFSET_LINE); 
    glPolygonOffset(-10.0, -10.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // disable culling
    glDisable(GL_CULL_FACE);


    /////////////////////////////////////////////////////////////////////////
    // RENDER LINES
    /////////////////////////////////////////////////////////////////////////

    // render all lines
    if (!m_displayListEdges.render(m_useDisplayList))
    {
        m_displayListEdges.begin(m_useDisplayList);
   
        glBegin(GL_TRIANGLES);
            vector<cEdge>::iterator i;
            for(i = m_edges->begin(); i != m_edges->end(); i++)
            {
                glVertex3dv(&(i)->m_vertex0->m_localPos(0));
                glVertex3dv(&(i)->m_vertex1->m_localPos(0));
                glVertex3dv(&(i)->m_vertex1->m_localPos(0));
            }
        glEnd();

        m_displayListEdges.end(true);
    }


    /////////////////////////////////////////////////////////////////////////
    // FINALIZE
    /////////////////////////////////////////////////////////////////////////
    
    if (m_cullingEnabled)
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
	}
	else
	{
		glDisable(GL_CULL_FACE);
	}

    // restore OpenGL preoperties
    glDisable(GL_POLYGON_OFFSET_LINE);
	glPolygonOffset(0.0, 0.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);
}


//===========================================================================
/*!
     Render the mesh itself.  This function is declared public to allow
     sharing of data among meshes, which is not possible given most
     implementations of 'protected'.  But it should only be accessed
     from within render() or derived versions of render().

     \fn        void cMesh::renderMesh(cRenderOptions& a_options)

	 \param		a_options  Rendering options.
*/
//===========================================================================
void cMesh::renderMesh(cRenderOptions& a_options)
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    // check if object contains any triangles or vertices
    if ((m_vertices->size() == 0) || (m_triangles->size() == 0))
    {
        return;
    }

    // initialize some OpenGL flags
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_EDGE_FLAG_ARRAY);
    glDisable(GL_COLOR_MATERIAL);

    if (m_useVertexArrays)
    {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);        
    }
    else
    {
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);    
    }

    //-----------------------------------------------------------------------
    // RENDER MATERIAL
    //-----------------------------------------------------------------------

    // render material properties if enabled
	if (m_useMaterialProperty && a_options.m_render_materials)
    {
        m_material->render(a_options);
    }

    //-----------------------------------------------------------------------
    // RENDER TEXTURE
    //-----------------------------------------------------------------------

    // render texture if enabled
    if ((m_texture != NULL) && (m_useTextureMapping) && (a_options.m_render_materials))
    {
		glActiveTexture(GL_TEXTURE1);

        if (m_useVertexArrays)
        {
            glEnableClientState(GL_TEXTURE_COORD_ARRAY);

            GLenum textureUnit = m_texture->getTextureUnit();
            glClientActiveTexture(textureUnit);
        }

        m_texture->render(a_options);
    }

        
    //-----------------------------------------------------------------------
    // RENDER VERTEX COLORS
    //-----------------------------------------------------------------------
    /*
    if vertex colors (m_useVertexColors) is enabled, we render the colors 
    defined for each individual vertex.
    
    if material properties (m_useMaterialProperty) has also been enabled, 
    then we combine vertex colors and materials together with OpenGL lighting 
    enabled.
    
    if material propeties are disabled then lighting is disabled and
    we use the pure color defined at each vertex to render the object
    */

    if (m_useVertexColors && a_options.m_render_materials)
    {
        // Clear the effects of material properties...
        if (m_useMaterialProperty)
        {
            // enable vertex colors
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glEnable(GL_COLOR_MATERIAL);
        }
        else
        {
            glDisable(GL_LIGHTING);            
        }

        // use vertex arrays if requested
        if (m_useVertexArrays)
        {
            glEnableClientState(GL_COLOR_ARRAY);
        }
    }
    else
    {
        glDisable(GL_COLOR_MATERIAL);
    }


    //-----------------------------------------------------------------------
    // FOR OBJECTS WITH NO DEFINED COLOR/MATERIAL SETTINGS
    //-----------------------------------------------------------------------
	/*
    A default color for objects that don't have vertex colors or
    material properties (otherwise they're invisible)...
	If texture mapping is enabled, then just turn off lighting
	*/

    if (((!m_useVertexColors) && (!m_useMaterialProperty)) && a_options.m_render_materials)
    {
		if (m_useTextureMapping)
		{
			glDisable(GL_LIGHTING);
		}
		else
		{
			glEnable(GL_COLOR_MATERIAL);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glColor4f(1.0,1.0,1.0,1.0);
		}
    }


    //-----------------------------------------------------------------------
    // DISPLAY LIST
    //-----------------------------------------------------------------------
    
    if (!m_displayList.render(m_useDisplayList))   
    {
        // if requested, begin creating display list
        m_displayList.begin(m_useDisplayList);


        //-------------------------------------------------------------------
        // RENDER ALL TRIANGLES
        //-------------------------------------------------------------------

        /////////////////////////////////////////////////////////////////////
        // RENDER TRIANGLES WITH VERTEX ARRAYS
        /////////////////////////////////////////////////////////////////////
        if (m_useVertexArrays)
        {
            // specify pointers to rendering arrays
            glEnableClientState(GL_TEXTURE_COORD_ARRAY);


            glVertexPointer(3, GL_DOUBLE, sizeof(cVertex), &(m_vertices->at(0).m_localPos));
            glNormalPointer(GL_DOUBLE, sizeof(cVertex), &(m_vertices->at(0).m_normal));
            glColorPointer(3, GL_FLOAT, sizeof(cVertex), m_vertices->at(0).m_color.pColor());
            glTexCoordPointer(2, GL_DOUBLE, sizeof(cVertex), &m_vertices->at(0).m_texCoord);

            // begin rendering triangles
            glBegin(GL_TRIANGLES);

            // render all active triangles
            vector<cTriangle>::iterator it;
		    for (it = m_triangles->begin(); it < m_triangles->end(); it++)
		    {
			    if ((*it).m_allocated)
                {
                    unsigned int index0 = (*it).m_indexVertex0;
                    unsigned int index1 = (*it).m_indexVertex1;
                    unsigned int index2 = (*it).m_indexVertex2;
                    glArrayElement(index0);
                    glArrayElement(index1);
                    glArrayElement(index2);
                }
            }

            // finalize rendering list of triangles
            glEnd();
        }


        /////////////////////////////////////////////////////////////////////
        // RENDER TRIANGLES USING CLASSIC OPENGL COMMANDS
        /////////////////////////////////////////////////////////////////////
        else
        {
            // begin rendering triangles
            glBegin(GL_TRIANGLES);

            // render all active triangles
            if (((!m_useTextureMapping) && (!m_useVertexColors)))
            {
			    vector<cTriangle>::iterator it;
			    for (it = m_triangles->begin(); it < m_triangles->end(); it++)
			    {
				    if ((*it).m_allocated)
				    {
					    // get pointers to vertices
					    cVertex* v0 = (*it).getVertex0();
					    cVertex* v1 = (*it).getVertex1();
					    cVertex* v2 = (*it).getVertex2();

					    // render vertex 0
					    glNormal3dv(&v0->m_normal(0) );
					    glVertex3dv(&v0->m_localPos(0) );

					    // render vertex 1
					    glNormal3dv(&v1->m_normal(0) );
					    glVertex3dv(&v1->m_localPos(0) );

					    // render vertex 2
					    glNormal3dv(&v2->m_normal(0) );
					    glVertex3dv(&v2->m_localPos(0) );
				    }
                }
            }
            else if ((m_useTextureMapping) && (!m_useVertexColors))
            {
			    vector<cTriangle>::iterator it;
			    for (it = m_triangles->begin(); it < m_triangles->end(); it++)
			    {
				    if ((*it).m_allocated)
				    {
					    // get pointers to vertices
					    cVertex* v0 = (*it).getVertex0();
					    cVertex* v1 = (*it).getVertex1();
					    cVertex* v2 = (*it).getVertex2();

					    // render vertex 0
					    glNormal3dv(&v0->m_normal(0) );
					    glMultiTexCoord2dv(GL_TEXTURE1, &v0->m_texCoord(0) );
					    glVertex3dv(&v0->m_localPos(0) );

					    // render vertex 1
					    glNormal3dv(&v1->m_normal(0) );
					    glMultiTexCoord2dv(GL_TEXTURE1, &v1->m_texCoord(0) );
					    glVertex3dv(&v1->m_localPos(0) );

					    // render vertex 2
					    glNormal3dv(&v2->m_normal(0) );
					    glMultiTexCoord2dv(GL_TEXTURE1, &v2->m_texCoord(0) );
					    glVertex3dv(&v2->m_localPos(0) );
				    }
			    }
            }
            else if ((!m_useTextureMapping) && (m_useVertexColors))
            {
			    vector<cTriangle>::iterator it;
			    for (it = m_triangles->begin(); it < m_triangles->end(); it++)
			    {
				    if ((*it).m_allocated)
				    {
					    // get pointers to vertices
					    cVertex* v0 = (*it).getVertex0();
					    cVertex* v1 = (*it).getVertex1();
					    cVertex* v2 = (*it).getVertex2();

					    // render vertex 0
					    glNormal3dv(&v0->m_normal(0) );
					    glColor4fv(v0->m_color.pColor());
					    glVertex3dv(&v0->m_localPos(0) );

					    // render vertex 1
					    glNormal3dv(&v1->m_normal(0) );
					    glColor4fv(v1->m_color.pColor());
					    glVertex3dv(&v1->m_localPos(0) );

					    // render vertex 2
					    glNormal3dv(&v2->m_normal(0) );
					    glColor4fv(v2->m_color.pColor());
					    glVertex3dv(&v2->m_localPos(0) );
				    }
                }
            }
            else if ((m_useTextureMapping) && (m_useVertexColors))
            {
			    vector<cTriangle>::iterator it;
			    for (it = m_triangles->begin(); it < m_triangles->end(); it++)
			    {
				    if ((*it).m_allocated)
				    {
					    // get pointers to vertices
					    cVertex* v0 = (*it).getVertex0();
					    cVertex* v1 = (*it).getVertex1();
					    cVertex* v2 = (*it).getVertex2();

					    // render vertex 0
					    glNormal3dv(&v0->m_normal(0) );
					    glColor4fv(v0->m_color.pColor());
					    glTexCoord2dv(&v0->m_texCoord(0) );
					    glVertex3dv(&v0->m_localPos(0) );

					    // render vertex 1
					    glNormal3dv(&v1->m_normal(0) );
					    glColor4fv(v1->m_color.pColor());
					    glTexCoord2dv(&v1->m_texCoord(0) );
					    glVertex3dv(&v1->m_localPos(0) );

					    // render vertex 2
					    glNormal3dv(&v2->m_normal(0) );
					    glColor4fv(v2->m_color.pColor());
					    glTexCoord2dv(&v2->m_texCoord(0) );
					    glVertex3dv(&v2->m_localPos(0) );
				    }
                }
            }

            // finalize rendering list of triangles
            glEnd();
        }

        //-------------------------------------------------------------------
        // FINALIZE DISPLAY LIST
        //-------------------------------------------------------------------

        // if being created, finalize display list
        m_displayList.end(true);
    }


    //-----------------------------------------------------------------------
    // RESTORE OPENGL
    //-----------------------------------------------------------------------

    // turn off any array variables I might have turned on...
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);
   
	if (m_useTextureMapping)
	{
		glActiveTexture(GL_TEXTURE1);
		glDisable(GL_TEXTURE_1D);
		glDisable(GL_TEXTURE_2D);
	}

    if (m_useVertexArrays)
    {
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }
}
