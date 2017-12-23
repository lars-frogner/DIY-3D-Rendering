#include "MeshTopology.hpp"
#include <cassert>
#include <sstream>

namespace Impact {
namespace Geometry3D {

MeshTopology::~MeshTopology()
{
	invalidateAdjacencyData();
}

void MeshTopology::initializeFaces(imp_uint n_faces, bool uses_texture_coordinates)
{
	_faces = arma::Mat<imp_uint>((uses_texture_coordinates)? 6 : 3, n_faces);
	_face_idx = 0;
}

void MeshTopology::reserveAdditionalFaces(imp_uint n_additional_faces)
{
	arma::Mat<imp_uint> additional_faces(_faces.n_rows, n_additional_faces);
	
	_face_idx = static_cast<imp_uint>(_faces.n_cols);

	_faces = arma::join_rows(_faces, additional_faces);
}

void MeshTopology::addFace(imp_uint i, imp_uint j, imp_uint k)
{
	assert(_faces.empty() || _faces.n_rows == 3);

    _faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({i, j, k}));
}

void MeshTopology::addFace(imp_uint i, imp_uint j, imp_uint k,
						   imp_uint l, imp_uint m, imp_uint n)
{
	assert(_faces.empty() || _faces.n_rows == 6);

    _faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({i, j, k, l, m, n}));
}

void MeshTopology::getFace(imp_uint face_idx,
						   imp_uint& i, imp_uint& j, imp_uint& k) const
{
	assert(face_idx < _faces.n_cols);

	i = _faces(0, face_idx);
	j = _faces(1, face_idx);
	k = _faces(2, face_idx);
}

void MeshTopology::getFace(imp_uint face_idx,
						   imp_uint& i, imp_uint& j, imp_uint& k,
						   imp_uint& l, imp_uint& m, imp_uint& n) const
{
	assert(face_idx < _faces.n_cols && _faces.n_rows == 6);

	i = _faces(0, face_idx);
	j = _faces(1, face_idx);
	k = _faces(2, face_idx);
	l = _faces(3, face_idx);
	m = _faces(4, face_idx);
	n = _faces(5, face_idx);
}

void MeshTopology::getTextureCoordinateIndices(imp_uint face_idx,
											   imp_uint& l, imp_uint& m, imp_uint& n) const
{
	assert(face_idx < _faces.n_cols && _faces.n_rows == 6);

	l = _faces(3, face_idx);
	m = _faces(4, face_idx);
	n = _faces(5, face_idx);
}

imp_uint MeshTopology::getTextureCoordinateIndexForVertexIndex(imp_uint face_idx, imp_uint vertex_idx) const
{
	assert(face_idx < _faces.n_cols && _faces.n_rows == 6);

	if (vertex_idx == _faces(0, face_idx))
		return _faces(3, face_idx);
	else if (vertex_idx == _faces(1, face_idx))
		return _faces(4, face_idx);
	else if (vertex_idx == _faces(2, face_idx))
		return _faces(5, face_idx);
	else
		throw;
}

void MeshTopology::setNextFace(imp_uint i, imp_uint j, imp_uint k)
{
	assert(_face_idx < _faces.n_cols || _faces.n_rows == 3);

    _faces(0, _face_idx) = i;
    _faces(1, _face_idx) = j;
    _faces(2, _face_idx) = k;

	_face_idx++;
}

void MeshTopology::setNextFace(imp_uint i, imp_uint j, imp_uint k,
							   imp_uint l, imp_uint m, imp_uint n)
{
	assert(_face_idx < _faces.n_cols || _faces.n_rows == 6);

    _faces(0, _face_idx) = i;
    _faces(1, _face_idx) = j;
    _faces(2, _face_idx) = k;
    _faces(3, _face_idx) = l;
    _faces(4, _face_idx) = m;
    _faces(5, _face_idx) = n;

	_face_idx++;
}

void MeshTopology::setFace(imp_uint face_idx,
						   imp_uint i, imp_uint j, imp_uint k)
{
	assert(face_idx < _faces.n_cols || _faces.n_rows == 3);

    _faces(0, face_idx) = i;
    _faces(1, face_idx) = j;
    _faces(2, face_idx) = k;
}

void MeshTopology::setFace(imp_uint face_idx,
						   imp_uint i, imp_uint j, imp_uint k,
						   imp_uint l, imp_uint m, imp_uint n)
{
	assert(face_idx < _faces.n_cols || _faces.n_rows == 6);

    _faces(0, face_idx) = i;
    _faces(1, face_idx) = j;
    _faces(2, face_idx) = k;
    _faces(3, face_idx) = l;
    _faces(4, face_idx) = m;
    _faces(5, face_idx) = n;
}

void MeshTopology::setFaceComponent(imp_uint component, imp_uint face_idx, imp_uint value)
{
	assert(component < _faces.n_rows);

	_faces(component, face_idx) = value;
}

void MeshTopology::removeFace(imp_uint face_idx)
{
	assert(face_idx < _faces.n_cols);

    _faces.shed_col(face_idx);
}

void MeshTopology::removeFacesContainingVertex(imp_uint vertex_idx)
{
	arma::uvec occurences = arma::unique(arma::find(_faces == vertex_idx)/_faces.n_rows);
    arma::uvec::const_iterator iter = occurences.end();
    
	while (iter != occurences.begin())
    {
        --iter;

        _faces.shed_col(*iter);
    }

    occurences = arma::find(_faces > vertex_idx);
    for (iter = occurences.begin(); iter != occurences.end(); ++iter)
    {
        _faces(*iter) += 1;
    }
}

void MeshTopology::removeFacesContainingVertex(imp_uint vertex_idx, arma::Mat<imp_float>& face_normals)
{
	arma::uvec occurences = arma::unique(arma::find(_faces == vertex_idx)/_faces.n_rows);
    arma::uvec::const_iterator iter = occurences.end();
    
	while (iter != occurences.begin())
    {
        --iter;

        _faces.shed_col(*iter);

		face_normals.shed_col(*iter);
    }

    occurences = arma::find(_faces > vertex_idx);
    for (iter = occurences.begin(); iter != occurences.end(); ++iter)
    {
        _faces(*iter) += 1;
    }
}

void MeshTopology::duplicateFaceIndicesForTextureCoordinates()
{
	assert(_faces.n_rows == 3);

	_faces = arma::join_cols(_faces, _faces);
}

void MeshTopology::generateOppositeFaces()
{
	assert(_faces.n_rows == 3);

	arma::Mat<imp_uint> faces_copy = _faces;

	faces_copy.swap_rows(0, 1);
	
	_faces = arma::join_rows(_faces, faces_copy);
}

void MeshTopology::generateAdjacencyData(imp_uint n_vertices)
{
	imp_uint n_faces = getNumberOfFaces();
	imp_uint face_idx;
    imp_uint i, j, k;

	HalfEdge* half_edge_1;
	HalfEdge* half_edge_2;
	HalfEdge* half_edge_3;

	HalfEdge* adjacent_half_edge;

	half_edge_map::iterator entry;

	std::pair<imp_uint, imp_uint> edge_pair;

	_outgoing_half_edges.resize(n_vertices);
	_first_bordering_half_edges.resize(n_faces);

    for (face_idx = 0; face_idx < n_faces; face_idx++)
	{
		i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

		// Add new half edges to the half edge map and get pointers to them

		edge_pair = std::make_pair(i, j);
		_half_edges[edge_pair] = HalfEdge();
		half_edge_1 = &(_half_edges[edge_pair]);
		
		edge_pair = std::make_pair(j, k);
		_half_edges[edge_pair] = HalfEdge();
		half_edge_2 = &(_half_edges[edge_pair]);
		
		edge_pair = std::make_pair(k, i);
		_half_edges[edge_pair] = HalfEdge();
		half_edge_3 = &(_half_edges[edge_pair]);
		
		// Set next half edge pointers
		half_edge_1->next_half_edge = half_edge_2;
		half_edge_2->next_half_edge = half_edge_3;
		half_edge_3->next_half_edge = half_edge_1;
		
		// Set bordered face
		half_edge_1->bordered_face_idx = face_idx;
		half_edge_2->bordered_face_idx = face_idx;
		half_edge_3->bordered_face_idx = face_idx;

		// Set half edge for bordered face (note that the convention of using the i -> j half edge is assumed by other procedures)
		_first_bordering_half_edges[face_idx] = half_edge_1;

		// Set vertices at the ends of the half edges
		half_edge_1->vertex_at_end_idx = j;
		half_edge_2->vertex_at_end_idx = k;
		half_edge_3->vertex_at_end_idx = i;

		// Set half edges for outgoing vertices (can be overwritten by later faces)
		_outgoing_half_edges[i] = half_edge_1;
		_outgoing_half_edges[j] = half_edge_2;
		_outgoing_half_edges[k] = half_edge_3;

		// Find half edge adjacent to the first half edge
		entry = _half_edges.find(std::make_pair(j, i));
		if (entry != _half_edges.end())
		{
			adjacent_half_edge = &(entry->second);

			// Set both half edges to be adjacent to each other
			half_edge_1->adjacent_half_edge = adjacent_half_edge;
			adjacent_half_edge->adjacent_half_edge = half_edge_1;
		}
		
		// Find half edge adjacent to the second half edge
		entry = _half_edges.find(std::make_pair(k, j));
		if (entry != _half_edges.end())
		{
			adjacent_half_edge = &(entry->second);
			
			// Set both half edges to be adjacent to each other
			half_edge_2->adjacent_half_edge = adjacent_half_edge;
			adjacent_half_edge->adjacent_half_edge = half_edge_2;
		}
		
		// Find half edge adjacent to the third half edge
		entry = _half_edges.find(std::make_pair(i, k));
		if (entry != _half_edges.end())
		{
			adjacent_half_edge = &(entry->second);
			
			// Set both half edges to be adjacent to each other
			half_edge_3->adjacent_half_edge = adjacent_half_edge;
			adjacent_half_edge->adjacent_half_edge = half_edge_3;
		}
	}

	_has_adjacency_data = true;
}

imp_uint MeshTopology::findConnectedVertices(imp_uint vertex_idx,
											 std::list<imp_uint>& connected_vertices) const
{
	assert(_has_adjacency_data);

	// Note that all neighbouring vertices cannot be detected if the mesh is not a manifold mesh

	// The connected vertices are ordered in the clockwise direction around the original vertex

	imp_uint n_connected_faces = 0;

	HalfEdge* original_half_edge = _outgoing_half_edges[vertex_idx];
	HalfEdge* outgoing_half_edge = original_half_edge;

	// Clockwise direction
	while (true)
	{
		// Add vertex at the end of outgoing half edge
		connected_vertices.push_back(outgoing_half_edge->vertex_at_end_idx);
		n_connected_faces++;

		if (outgoing_half_edge->adjacent_half_edge) // Is there another connected half edge in the clockwise direction?
		{
			// Move to next outgoing half edge in the clockwise direction
			outgoing_half_edge = outgoing_half_edge->adjacent_half_edge->next_half_edge;

			// Exit if we have made it all the way around the vertex
			if (outgoing_half_edge == original_half_edge)
				return n_connected_faces;
		}
		else
		{
			// Move to the first outgoing half-edge in the counter-clockwise direction from the original half edge
			outgoing_half_edge = original_half_edge->next_half_edge->next_half_edge->adjacent_half_edge;

			// If this half edge doesn't exist, we are done
			if (!outgoing_half_edge)
			{
				// Make sure to include the final vertex (which is only connected by an incoming half edge)
				connected_vertices.push_front(original_half_edge->next_half_edge->vertex_at_end_idx);
				return n_connected_faces;
			}

			// Move to counter-clockwise loop
			break;
		}
	}

	// Counter-clockwise direction
	while (true)
	{
		// Add vertex at the end of outgoing half edge
		connected_vertices.push_front(outgoing_half_edge->vertex_at_end_idx);
		n_connected_faces++;
		
		// If the next outgoing half-edge in the counter-clockwise direction doesn't exist, we are done
		if (!outgoing_half_edge->next_half_edge->next_half_edge->adjacent_half_edge)
		{
			// Make sure to include the final vertex (which is only connected by an incoming half edge)
			connected_vertices.push_front(outgoing_half_edge->next_half_edge->vertex_at_end_idx);
			return n_connected_faces;
		}
		
		// Otherwise, move to this half edge
		outgoing_half_edge = outgoing_half_edge->next_half_edge->next_half_edge->adjacent_half_edge;
	}
}

void MeshTopology::findConnectedFacesAndVertices(imp_uint vertex_idx,
												 std::list<imp_uint>& connected_faces,
												 std::list<imp_uint>& connected_vertices) const
{
	assert(_has_adjacency_data);

	// Note that all neighbouring faces cannot be detected if the mesh is not a manifold mesh
	
	// The connected faces and vertices are ordered in the clockwise direction around the original vertex

	HalfEdge* original_half_edge = _outgoing_half_edges[vertex_idx];
	HalfEdge* outgoing_half_edge = original_half_edge;

	// Clockwise direction
	while (true)
	{
		// Add face bordered by the outgoing half edge
		connected_faces.push_back(outgoing_half_edge->bordered_face_idx);

		// Add vertex at the end of outgoing half edge
		connected_vertices.push_back(outgoing_half_edge->vertex_at_end_idx);

		if (outgoing_half_edge->adjacent_half_edge) // Is there another connected half edge in the clockwise direction?
		{
			// Move to next outgoing half edge in the clockwise direction
			outgoing_half_edge = outgoing_half_edge->adjacent_half_edge->next_half_edge;

			// Exit if we have made it all the way around the vertex
			if (outgoing_half_edge == original_half_edge)
				return;
		}
		else
		{
			// Move to the first outgoing half-edge in the counter-clockwise direction from the original half edge
			outgoing_half_edge = original_half_edge->next_half_edge->next_half_edge->adjacent_half_edge;

			// If this half edge doesn't exist, we are done
			if (!outgoing_half_edge)
			{
				// Make sure to include the final vertex (which is only connected by an incoming half edge)
				connected_vertices.push_front(original_half_edge->next_half_edge->vertex_at_end_idx);
				return;
			}

			// Move to counter-clockwise loop
			break;
		}
	}

	// Counter-clockwise direction
	while (true)
	{
		// Add face bordered by outgoing half edge
		connected_faces.push_front(outgoing_half_edge->bordered_face_idx);

		// Add vertex at the end of outgoing half edge
		connected_vertices.push_front(outgoing_half_edge->vertex_at_end_idx);
		
		// If the next outgoing half-edge in the counter-clockwise direction doesn't exist, we are done
		if (!outgoing_half_edge->next_half_edge->next_half_edge->adjacent_half_edge)
		{
			// Make sure to include the final vertex (which is only connected by an incoming half edge)
			connected_vertices.push_front(outgoing_half_edge->next_half_edge->vertex_at_end_idx);
			return;
		}
		
		// Otherwise, move to this half edge
		outgoing_half_edge = outgoing_half_edge->next_half_edge->next_half_edge->adjacent_half_edge;
	}
}

void MeshTopology::findAdjacentFaces(imp_uint face_idx,
									 imp_uint adjacent_faces[3]) const
{
	assert(_has_adjacency_data);

	// Note that adjacent_faces[*] is set to face_idx if the relevant adjacent face doesn't exist

	const HalfEdge* i_j_half_edge = _first_bordering_half_edges[face_idx];
	const HalfEdge* adjacent_half_edge = i_j_half_edge->adjacent_half_edge;

	if (adjacent_half_edge)
	{
		adjacent_faces[0] = adjacent_half_edge->bordered_face_idx;
	}
	else
	{
		adjacent_faces[0] = face_idx;
	}

	adjacent_half_edge = i_j_half_edge->next_half_edge->adjacent_half_edge;

	if (adjacent_half_edge)
	{
		adjacent_faces[1] = adjacent_half_edge->bordered_face_idx;
	}
	else
	{
		adjacent_faces[1] = face_idx;
	}

	adjacent_half_edge = i_j_half_edge->next_half_edge->next_half_edge->adjacent_half_edge;

	if (adjacent_half_edge)
	{
		adjacent_faces[2] = adjacent_half_edge->bordered_face_idx;
	}
	else
	{
		adjacent_faces[2] = face_idx;
	}
}

void MeshTopology::findAdjacentFaces(imp_uint face_idx,
									 imp_uint adjacent_faces[3],
									 imp_uint opposite_vertices[3]) const
{
	assert(_has_adjacency_data);

	// Note that adjacent_faces[*] is set to face_idx if the relevant opposite face doesn't exist

	const HalfEdge* i_j_half_edge = _first_bordering_half_edges[face_idx];
	const HalfEdge* adjacent_half_edge = i_j_half_edge->adjacent_half_edge;

	if (adjacent_half_edge)
	{
		adjacent_faces[0] = adjacent_half_edge->bordered_face_idx;
		opposite_vertices[0] = adjacent_half_edge->next_half_edge->vertex_at_end_idx;
	}
	else
	{
		adjacent_faces[0] = face_idx;
	}

	adjacent_half_edge = i_j_half_edge->next_half_edge->adjacent_half_edge;

	if (adjacent_half_edge)
	{
		adjacent_faces[1] = adjacent_half_edge->bordered_face_idx;
		opposite_vertices[1] = adjacent_half_edge->next_half_edge->vertex_at_end_idx;
	}
	else
	{
		adjacent_faces[1] = face_idx;
	}

	adjacent_half_edge = i_j_half_edge->next_half_edge->next_half_edge->adjacent_half_edge;

	if (adjacent_half_edge)
	{
		adjacent_faces[2] = adjacent_half_edge->bordered_face_idx;
		opposite_vertices[2] = adjacent_half_edge->next_half_edge->vertex_at_end_idx;
	}
	else
	{
		adjacent_faces[2] = face_idx;
	}
}

void MeshTopology::removeTextureCoordinateIndices()
{
	assert(_faces.n_rows == 6);

	_faces.shed_rows(3, 5);
}

void MeshTopology::invalidateAdjacencyData()
{
	if (!_has_adjacency_data)
		return;
	
	_half_edges.clear();
	_outgoing_half_edges.clear();
	_first_bordering_half_edges.clear();

	_has_adjacency_data = false;
}

bool MeshTopology::hasTextureCoordinateIndices() const
{
	return _faces.n_rows == 6;
}

bool MeshTopology::hasAdjacencyData() const
{
	return _has_adjacency_data;
}

imp_uint MeshTopology::getNumberOfFaces() const
{
    return static_cast<imp_uint>(_faces.n_cols);
}

std::string MeshTopology::getFacesString() const
{
    std::ostringstream string_stream;
    string_stream << _faces;
    return string_stream.str();
}

HalfEdge::HalfEdge()
	: next_half_edge(nullptr),
	  adjacent_half_edge(nullptr) {}

} // Geometry3D
} // Impact
