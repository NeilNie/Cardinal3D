
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"


Vec3 face_normal(Halfedge_Mesh::FaceRef f) {
    auto v1 = f->halfedge()->vertex()->pos - f->halfedge()->twin()->vertex()->pos;
    auto v2 = f->halfedge()->next()->vertex()->pos - f->halfedge()->next()->twin()->vertex()->pos;
    return cross(v1, v2).normalize();
}

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

// this method assumes that the start and end edges point to the same direction.
std::vector<Halfedge_Mesh::HalfedgeRef> collect_half_edges_between(Halfedge_Mesh::HalfedgeRef start, Halfedge_Mesh::HalfedgeRef end) {

    std::vector<Halfedge_Mesh::HalfedgeRef> half_edges;
    auto h = start->next();
    do {
        h->vertex()->halfedge() = h;
        half_edges.push_back(h);
        h = h->next();
    } while (h != end->twin());
    return half_edges;
}

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    std::vector<FaceRef> to_erase;
    FaceRef face = new_face();
    std::vector<HalfedgeRef> boundary_half_edges;
    std::vector<HalfedgeRef> half_edges_to_erase;

    // 1. get all faces incident on the vertex, we need to erase
    // 2. collect all half edges along the edges
    auto h = v->halfedge();
    do {
        auto partial_half_edges = collect_half_edges_between(h->twin()->next(), h);
        boundary_half_edges.insert(boundary_half_edges.end(), partial_half_edges.begin(), partial_half_edges.end());
        half_edges_to_erase.push_back(h);
        half_edges_to_erase.push_back(h->twin());
        to_erase.push_back(h->face());
        h = h->twin()->next();
    } while (h != v->halfedge());
    std::reverse(boundary_half_edges.begin(), boundary_half_edges.end());

    // Now, link the boundary half edges to form the new face
    for (size_t i = 0; i < boundary_half_edges.size(); ++i) {
        boundary_half_edges[i]->face() = face;
        if (i < boundary_half_edges.size() - 1)
            boundary_half_edges[i]->next() = boundary_half_edges[i + 1];
        else
            boundary_half_edges[i]->next() = boundary_half_edges[0];
    }

    // Update the new face to point to one of the boundary halfedges
    face->halfedge() = boundary_half_edges[0];

    // erase all of the faces
    for (auto f : to_erase)
        erase(f);
    for (auto halfedge: half_edges_to_erase) {
        erase(halfedge);
        erase(halfedge->edge());
    }

    erase(v);

    return face;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

std::vector<Halfedge_Mesh::HalfedgeRef> get_all_half_edges_of_vertex(Halfedge_Mesh::VertexRef v) {

    std::vector<Halfedge_Mesh::HalfedgeRef> h_edges;
    auto current_half_edge = v->halfedge();
    int counter = 0;

    do {
        if (counter % 2 != 0) {
            current_half_edge = current_half_edge->next();
            counter += 1;
            continue;
        }
        h_edges.push_back(current_half_edge);
        current_half_edge = current_half_edge->twin();
        counter += 1;
    } while (current_half_edge != v->halfedge());

    return h_edges;
}

size_t num_of_edges(Halfedge_Mesh::FaceRef f) {

    auto current_half_edge = f->halfedge();
    size_t counter = 0;
    do {
        counter += 1;
        current_half_edge = current_half_edge->next();
    } while (current_half_edge != f->halfedge());

    return counter;
}

void reassign_erase_for_collapse_edge(Halfedge_Mesh* mesh, Halfedge_Mesh::HalfedgeRef h0, Halfedge_Mesh::VertexRef new_v) {

    Halfedge_Mesh::FaceRef face = h0->face();

    Halfedge_Mesh::HalfedgeRef h1 = h0->next();
    Halfedge_Mesh::HalfedgeRef h2 = h0->next()->next();
    Halfedge_Mesh::HalfedgeRef h3 = h0->next()->next()->twin();
    Halfedge_Mesh::HalfedgeRef h4 = h0->next()->twin();

    Halfedge_Mesh::VertexRef v = h2->vertex();

    Halfedge_Mesh::EdgeRef e1 = h1->edge();
    Halfedge_Mesh::EdgeRef e2 = h2->edge();

    h3->twin() = h4;
    h4->twin() = h3;
    h4->edge() = e2;
    e2->halfedge() = h4;
    if (v->halfedge() == h2)
        v->halfedge() = h4;
    new_v->halfedge() = h3;

    mesh->erase(face);
    mesh->erase(h1);
    mesh->erase(h2);
    mesh->erase(e1);
}

bool can_collapse_edge(Halfedge_Mesh::EdgeRef e) {

    // boundary check
    if (e->on_boundary())
        return false;

    // =======
    auto h0 = e->halfedge();
    auto h2 = h0->next();
    auto h4 = h2->next();
    auto h1 = h0->twin();
    auto h3 = h1->next();
    auto h5 = h3->next();
    auto e2 = h2->edge();
    auto e4 = h4->edge();
    auto e1 = h5->edge();
    auto e3 = h3->edge();

    // two cases:
    // the two vertices v1 and v2 are the same
    // or the two triangles are on top of each other
    if (e->halfedge()->vertex() == e->halfedge()->twin()->vertex() || e3 == e4 || e1 == e2) {
        return false;
    }

    // get the neighboring vertices of v0
    auto v0_h = h0; 
    std::set<Halfedge_Mesh::VertexRef> v0_neighbors;
    do {
        v0_neighbors.insert(v0_h->twin()->vertex());
        v0_h = v0_h->twin()->next();
    } while (v0_h != h0);

    // get the neighboring vertices for v1
    auto v1_h = h1; 
    std::set<Halfedge_Mesh::VertexRef> v1_neighbors;
    do {
        v1_neighbors.insert(v1_h->twin()->vertex());
        v1_h = v1_h->twin()->next();
    } while (v1_h != h1);

    std::set<Halfedge_Mesh::VertexRef> shared_neighbors;
    std::set_intersection(
        v0_neighbors.begin(), v0_neighbors.end(),
        v1_neighbors.begin(), v1_neighbors.end(),
        std::inserter(shared_neighbors, shared_neighbors.begin()));

    // counting the joint neighbour vertices of the two merging vertices (there must be exactly two)
    if (shared_neighbors.size() != 2) {
        return false;
    }

    return true;
}


std::unordered_map<Halfedge_Mesh::FaceRef, Vec3> all_face_normals(Halfedge_Mesh::VertexRef v) {

    std::unordered_map<Halfedge_Mesh::FaceRef, Vec3> og_face_normals;
    Halfedge_Mesh::HalfedgeRef h = v->halfedge();
    do {
        og_face_normals[h->face()] = face_normal(h->face());
        h = h->twin()->next();
    } while (h != v->halfedge());

    return og_face_normals;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/

std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    if (!can_collapse_edge(e)) {
        return std::nullopt;
    }

    bool double_triangle = num_of_edges(e->halfedge()->face()) == 3 && num_of_edges(e->halfedge()->twin()->face()) == 3;

    auto v1 = e->halfedge()->vertex();
    auto v2 = e->halfedge()->twin()->vertex();
    auto v3 = new_vertex();
    v3->pos = (v1->pos + v2->pos) / 2;

    auto face_normals_before = all_face_normals(v1);
    auto r2 = all_face_normals(v2);
    face_normals_before.insert(r2.begin(), r2.end());

    // reassign halfedges for all vertex v1 and v2
    auto edges_v1 = get_all_half_edges_of_vertex(v1);
    auto edges_v2 = get_all_half_edges_of_vertex(v2);
    std::vector<Halfedge_Mesh::HalfedgeRef> all_half_edges;
    std::vector<Halfedge_Mesh::VertexRef > og_half_edge_vertices;
    all_half_edges.reserve(edges_v1.size() + edges_v2.size());
    all_half_edges.insert(all_half_edges.end(), edges_v1.begin(), edges_v1.end());
    all_half_edges.insert(all_half_edges.end(), edges_v2.begin(), edges_v2.end());

    for (auto halfedge: all_half_edges) {
        og_half_edge_vertices.push_back(halfedge->vertex());
        halfedge->vertex() = v3;
    }
    v3->halfedge() = all_half_edges[0];

    // collapse overlapping edges
    if (double_triangle) {
        reassign_erase_for_collapse_edge(this, e->halfedge(), v3);
        reassign_erase_for_collapse_edge(this, e->halfedge()->twin(), v3);

    } else {
        // reassign faces
        e->halfedge()->face()->halfedge() = e->halfedge()->next();
        e->halfedge()->twin()->face()->halfedge() = e->halfedge()->twin()->next();

        // remove the collapsed edge
        HalfedgeRef h1 = e->halfedge();
        do {
            h1 = h1->next();
        } while (h1->next() != e->halfedge());
        HalfedgeRef h2 = e->halfedge()->twin();
        do {
            h2 = h2->next();
        } while (h2->next() != e->halfedge()->twin());
        h1->next() = e->halfedge()->next();
        h2->next() = e->halfedge()->twin()->next();
    }

    Halfedge_Mesh::erase(e);
    Halfedge_Mesh::erase(e->halfedge());
    Halfedge_Mesh::erase(e->halfedge()->twin());
    Halfedge_Mesh::erase(v1);
    Halfedge_Mesh::erase(v2);

    return v3;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    // TODO: required
    // Get the two faces
    FaceRef f0 = e->halfedge()->face();
    FaceRef f1 = e->halfedge()->twin()->face();

    // return immediately if on boundary
    if(f0->is_boundary() || f1->is_boundary())
    {
        printf("skipping flip because on boundary\n");
        return std::nullopt;
    }

    // HALF EDGES
    HalfedgeRef h0 = e->halfedge(), h1 = h0->next(), h2 = h1->next();           // triangle 1
    HalfedgeRef h3 = h0->twin(), h4 = h3->next(), h5 = h4->next();              // triangle 2
    HalfedgeRef h6 = h1->twin(), h7 = h2->twin(), h8 = h4->twin(), h9 = h5->twin(); // outside

    // reset h4 and h7 to support general polygon
    while(h4->next() != h0)
        h4 = h4->next();
    while(h7->next() != h1)
        h7 = h7->next();

    // VERTICES
    VertexRef v0 = h0->vertex(), v1 = h3->vertex(), v2 = h8->vertex(), v3 = h6->vertex();

    // EDGES
    EdgeRef e1 = h5->edge(), e2 = h4->edge(), e3 = h2->edge(), e4 = h1->edge();

    h0->set_neighbors(h1, h3, v2, e, f0);
    h1->set_neighbors(h2, h7, v3, e3, f0);
    h2->set_neighbors(h0, h8, v0, e2, f0);
    h3->set_neighbors(h4, h0, v3, e, f1);
    h4->set_neighbors(h5, h9, v2, e1, f1);
    h5->set_neighbors(h3, h6, v1, e4, f1);
    h6->set_neighbors(h6->next(), h5, v3, e4, h6->face());
    h7->set_neighbors(h7->next(), h1, v0, e3, h7->face());
    h8->set_neighbors(h8->next(), h2, v2, e2, h8->face());
    h9->set_neighbors(h9->next(), h4, v1, e1, h9->face());

    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h4;
    v3->halfedge() = h3;

    e->halfedge() = h0;
    e1->halfedge() = h4;
    e2->halfedge() = h2;
    e3->halfedge() = h1;
    e4->halfedge() = h5;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    // TODO: required
    // left side surface vertices
    FaceRef f0 = e->halfedge()->face();
    FaceRef f1 = e->halfedge()->twin()->face();

    // return immediately if on boundary
    if(f0->is_boundary() || f1->is_boundary())
    {
        printf("skipping split because on boundary\n");
        return std::nullopt;
    }

    // edge on boundary:
    bool edge_on_boundary = e->on_boundary();

    // ---- Existing elements ----
    HalfedgeRef h0 = e->halfedge(), h1 = h0->next(), h2 = h1->next();           // triangle 1
    HalfedgeRef h3 = h0->twin(), h4 = h3->next(), h5 = h4->next();              // triangle 2
    HalfedgeRef h6 = h1->twin(), h7 = h2->twin(), h8 = h4->twin(), h9 = h5->twin(); // outside

    VertexRef v0 = h0->vertex(), v1 = h3->vertex(), v2 = h8->vertex(), v3 = h6->vertex();

    EdgeRef e1 = h5->edge(), e2 = h4->edge(), e3 = h2->edge(), e4 = h1->edge();

    // ---- allocation ----
    HalfedgeRef h10 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h11 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h12 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h13 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h14 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h15 = Halfedge_Mesh::new_halfedge();

    FaceRef f2 = Halfedge_Mesh::new_face(), f3 = Halfedge_Mesh::new_face();

    VertexRef v4 = Halfedge_Mesh::new_vertex();

    EdgeRef e5 = Halfedge_Mesh::new_edge(), e6 = Halfedge_Mesh::new_edge(), e7 = Halfedge_Mesh::new_edge();

    HalfedgeRef h3_prev = h3;
    if(edge_on_boundary) {
        // the edge that used to have h3 as its next now needs to have h10 as its next
        while(h3_prev->twin()->next() != h3)
            h3_prev = h3_prev->twin()->next();
        h3_prev = h3_prev->twin(); // now h3_prev has h3 as its next
    }

    // ---- Assignments ----
    // HalfedgeRef      next,   twin,   vertex, edge,   face
    h0->set_neighbors(  h13,    h3,     v0,     e,      f0);
    h1->set_neighbors(  h12,    h6,     v1,     e4,     f3);
    h2->set_neighbors(  h0,     h7,     v3,     e3,     f0);
    h11->set_neighbors( h1,     h10,    v4,     e5,     f3);
    h12->set_neighbors( h11,    h13,    v3,     e6,     f3);
    h13->set_neighbors( h2,     h12,    v4,     e6,     f0);

    v1->halfedge() = h1;
    v4->halfedge() = h3;

    e5->halfedge() = h10;
    e6->halfedge() = h12;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f3->halfedge() = h1;

    if(edge_on_boundary) {
        h3->set_neighbors(h3->next(), h0, v4, e, f1);
        h3_prev->next() = h10;
        h10->set_neighbors(h3, h11, v1, e5, f1);
    } else {
        h3->set_neighbors(h4, h0, v4, e, f1);
        h4->set_neighbors(h14, h8, v0, e2, f1);
        h5->set_neighbors(h10, h9, v2, e1, f2);
        h10->set_neighbors(h15, h11, v1, e5, f2);
        h14->set_neighbors(h3, h15, v2, e7, f1);
        h15->set_neighbors(h5, h14, v4, e7, f2);

        e7->halfedge() = h14;
        f2->halfedge() = h5;
    }

    // remember to set the position of the new vertex
    v4->pos = (v0->pos + v1->pos) / 2.0;

    return v4;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

std::vector<Halfedge_Mesh::VertexRef> collect_vertices(Halfedge_Mesh::FaceRef f) {

    std::vector<Halfedge_Mesh::VertexRef> vertices;
    Halfedge_Mesh::HalfedgeRef h = f->halfedge();
    do {
        vertices.push_back(h->vertex());
        h = h->next();
    } while (h != f->halfedge());
    return vertices;
}

Halfedge_Mesh::HalfedgeRef half_edge_from_vertex(Halfedge_Mesh::FaceRef f, Halfedge_Mesh::VertexRef v) {

    Halfedge_Mesh::HalfedgeRef h = f->halfedge();
    do {
        if (h->vertex()->id() == v->id())
            break;
        h = h->next();
    } while (h != f->halfedge());
    return h;
}

Halfedge_Mesh::HalfedgeRef half_edge_to_vertex_on_face(Halfedge_Mesh::FaceRef f, Halfedge_Mesh::VertexRef v) {

    Halfedge_Mesh::HalfedgeRef h = f->halfedge();
    do {
        if (h->next()->vertex()->id() == v->id())
            break;
        h = h->next();
    } while (h != f->halfedge());
    return h;
}

void reset_halfedge_face(Halfedge_Mesh::FaceRef f) {

    Halfedge_Mesh::HalfedgeRef h = f->halfedge();
    do {
        h->face() = f;
        h = h->next();
    } while (h != f->halfedge());
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    Halfedge_Mesh::FaceRef new_face = Halfedge_Mesh::new_face();
    auto og_vertices = collect_vertices(f);

    // for every original vertex, we need to create the following 8 new elements
    // 1. a new face
    // 2. a new vertex
    // 3. a new edge from old vertex to new vertex
    // 4. a new edge from new vertex to next Vertex
    // 5-8. half edge for each of the two new edges
    std::vector<Halfedge_Mesh::FaceRef> faces;
    std::vector<Halfedge_Mesh::VertexRef> new_vertices;
    std::vector<Halfedge_Mesh::EdgeRef> edges_to_old;
    std::vector<Halfedge_Mesh::EdgeRef> edges_to_next_v;
    std::vector<Halfedge_Mesh::HalfedgeRef> h_edges_v_to_old;
    std::vector<Halfedge_Mesh::HalfedgeRef> h_edges_v_to_next;
    std::vector<Halfedge_Mesh::HalfedgeRef> h_edges_v_from_old;
    std::vector<Halfedge_Mesh::HalfedgeRef> h_edges_v_from_next;

    for (size_t i = 0; i < og_vertices.size(); i++) {
        faces.push_back(Halfedge_Mesh::new_face());
        new_vertices.push_back(Halfedge_Mesh::new_vertex());
        edges_to_old.push_back(Halfedge_Mesh::new_edge());
        edges_to_next_v.push_back(Halfedge_Mesh::new_edge());
        h_edges_v_to_old.push_back(Halfedge_Mesh::new_halfedge());
        h_edges_v_to_next.push_back(Halfedge_Mesh::new_halfedge());
        h_edges_v_from_old.push_back(Halfedge_Mesh::new_halfedge());
        h_edges_v_from_next.push_back(Halfedge_Mesh::new_halfedge());
    }
    
    std::vector<Halfedge_Mesh::HalfedgeRef> hes_from_og_vertex;
    std::vector<Halfedge_Mesh::HalfedgeRef> hes_to_og_vertex;
    for (size_t i = 0; i < og_vertices.size(); i++) {
        hes_from_og_vertex.push_back(half_edge_from_vertex(f, og_vertices[i]));
        hes_to_og_vertex.push_back(half_edge_to_vertex_on_face(f, og_vertices[i]));
    }

    for (size_t i = 0; i < og_vertices.size(); i++) {

        size_t next_i = i + 1;
        if (next_i == og_vertices.size())
            next_i = 0;
        size_t prev_i = (i - 1 + og_vertices.size()) % og_vertices.size();

        Halfedge_Mesh::FaceRef new_small_face = faces[i];
        Halfedge_Mesh::VertexRef v = new_vertices[i];
        Halfedge_Mesh::EdgeRef edge_to_old = edges_to_old[i];
        Halfedge_Mesh::EdgeRef edge_to_next = edges_to_next_v[i];
        Halfedge_Mesh::HalfedgeRef h_v_to_old = h_edges_v_to_old[i];
        Halfedge_Mesh::HalfedgeRef h_v_to_next = h_edges_v_to_next[i];
        Halfedge_Mesh::HalfedgeRef h_v_from_old = h_edges_v_from_old[i];
        Halfedge_Mesh::HalfedgeRef h_v_from_next = h_edges_v_from_next[i];

        auto he_from_og_vertex = hes_from_og_vertex[i];
        auto he_to_og_vertex = hes_to_og_vertex[i];
        auto n_vertex = new_vertices[next_i];

        //                           next                         twin            vertex          edge          face
        h_v_to_old->set_neighbors   (he_from_og_vertex,           h_v_from_old,   v,              edge_to_old,  new_small_face);
        h_v_from_old->set_neighbors (h_edges_v_from_next[prev_i], h_v_to_old,     og_vertices[i], edge_to_old,  faces[prev_i]);
        h_v_to_next->set_neighbors  (h_edges_v_to_next[next_i],   h_v_from_next,  v,              edge_to_next, new_face);
        h_v_from_next->set_neighbors(h_v_to_old,                  h_v_to_next,    n_vertex,       edge_to_next, new_small_face);

        // reset next() and face() for old edges
        he_to_og_vertex->next() = h_v_from_old;
        he_from_og_vertex->next() = h_edges_v_from_old[next_i];
        he_from_og_vertex->face() = new_small_face;
        he_to_og_vertex->face() = faces[prev_i];

        edge_to_old->halfedge() = h_v_to_old;
        edge_to_next->halfedge() = h_v_to_next;

        v->halfedge() = h_v_to_old;
        
        v->pos = og_vertices[i]->pos;

        // small face halfedge
        new_small_face->halfedge() = he_from_og_vertex;
    }

    new_face->halfedge() = h_edges_v_to_next[0];
    Halfedge_Mesh::erase(f);

    return new_face;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    int N = new_halfedges.size();

    for(size_t i = 0; i < new_halfedges.size(); i++) {
        Vec3 pi = start_positions[i]; // get the original vertex position corresponding to vertex i
        // calculate tangent (the average of the directions to prev and next)
        Vec3 prev_position = start_positions[(i + N - 1) % N];
        Vec3 next_position = start_positions[(i + 1) % N];
        Vec3 i_to_prev = (prev_position - pi).normalize();
        Vec3 i_to_next = (next_position - pi).normalize();
        Vec3 tangent = (i_to_prev + i_to_next) / (sqrt(2) / 2.0); // normalized
        new_halfedges[i]->vertex()->pos = pi + normal_offset * face->normal() + tangent_offset * tangent;
    }
}


std::vector<Halfedge_Mesh::HalfedgeRef> collect_all_half_edges(Halfedge_Mesh::FaceRef f) {
    
    std::vector<Halfedge_Mesh::HalfedgeRef> half_edges;

    Halfedge_Mesh::HalfedgeRef h = f->halfedge();
    do {
        half_edges.push_back(h);
        h = h->next();
    } while (h != f->halfedge());

    return half_edges;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {

        auto og_half_edges = collect_all_half_edges(f);
        if (og_half_edges.size() == 3)
            continue;
        
        std::vector<Halfedge_Mesh::FaceRef> new_faces = {};
        std::vector<Halfedge_Mesh::HalfedgeRef> half_edges_from_v = {og_half_edges[0]};
        for (size_t i = 1; i < og_half_edges.size() - 2; i++) {
            new_faces.push_back(Halfedge_Mesh::new_face());
            half_edges_from_v.push_back(Halfedge_Mesh::new_halfedge());
        }
        new_faces.push_back(f);

        for (size_t i = 1; i < og_half_edges.size() - 2; i++)
        {
            // create a new edge, two new half edges a new face
            Halfedge_Mesh::EdgeRef edge = new_edge();
            Halfedge_Mesh::HalfedgeRef from_v = half_edges_from_v[i];
            Halfedge_Mesh::HalfedgeRef to_v = new_halfedge();

            // set two halfedges
            to_v->set_neighbors(half_edges_from_v[i - 1], from_v, og_half_edges[i + 1]->vertex(), edge, new_faces[i - 1]);
            from_v->set_neighbors(og_half_edges[i + 1], to_v, og_half_edges[0]->vertex(), edge, new_faces[i]);

            // reset old half edges next and face
            og_half_edges[i]->next() = to_v;
            og_half_edges[i]->face() = new_faces[i - 1];

            new_faces[i - 1]->halfedge() = to_v;
            edge->halfedge() = from_v;
        }

        // the corner cases for the first and last triangle on the face
        half_edges_from_v[0]->face() = new_faces[0];
        new_faces.back()->halfedge() = og_half_edges.back();
        og_half_edges.back()->next() = half_edges_from_v.back();
    }
}


/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!

    for (auto v = vertices_begin(); v != vertices_end(); v++)
        v->new_pos = v->pos;

    for (auto e = edges_begin(); e != edges_end(); e++)
        e->new_pos = (e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 2.0;

    for (auto f = faces_begin(); f != faces_end(); f++) {
        float count = 0.f;
        Vec3 vec = Vec3();
        HalfedgeRef h = f->halfedge();
        do {
            vec += h->vertex()->pos;
            count += 1;
            h = h->next();
        } while (h != f->halfedge());
        f->new_pos = vec / count;
    }
}

size_t degrees(Halfedge_Mesh::VertexRef v) {

    size_t degrees = 0;
    Halfedge_Mesh::HalfedgeRef h = v->halfedge();
    do {
        degrees += 1;
        h = h->twin()->next();
    } while (h != v->halfedge());
    return degrees;
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for (auto f = faces_begin(); f != faces_end(); f++) {
        float count = 0.f;
        Vec3 vec = Vec3();
        HalfedgeRef h = f->halfedge();
        do {
            vec += h->vertex()->pos;
            count += 1;
            h = h->next();
        } while (h != f->halfedge());
        f->new_pos = vec / count;
    }

    // Edges
    for (auto e = edges_begin(); e != edges_end(); e++)
        e->new_pos = (e->halfedge()->face()->new_pos + e->halfedge()->twin()->face()->new_pos + 
                      e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 4.0;

    // Vertices
    for (auto v = vertices_begin(); v != vertices_end(); v++) {
        
        float n = (float)degrees(v);
        Vec3 Q = Vec3(), R = Vec3(), S = Vec3();
        
        Halfedge_Mesh::HalfedgeRef h = v->halfedge();
        do {
            Q += h->face()->new_pos;
            R += (h->vertex()->pos + h->twin()->vertex()->pos) / 2.0;
            h = h->twin()->next();
        } while (h != v->halfedge());

        Q /= n;
        R /= n;

        v->new_pos = (Q + 2 * R + (n - 3) * v->pos) / n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}


void print_matrix(Mat4 mat) {

    printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n", 
    mat[0][0], mat[1][0], mat[2][0], mat[3][0],
    mat[0][1], mat[1][1], mat[2][1], mat[3][1],
    mat[0][2], mat[1][2], mat[2][2], mat[3][2],
    mat[0][3], mat[1][3], mat[2][3], mat[3][3]
    );
}


/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.

        Mat4 endpoints_sum = vertex_quadrics[e->halfedge()->vertex()] + vertex_quadrics[e->halfedge()->twin()->vertex()];

        Mat4 A = Mat4(  Vec4(endpoints_sum[0][0], endpoints_sum[0][1], endpoints_sum[0][2], 0.f),
                        Vec4(endpoints_sum[1][0], endpoints_sum[1][1], endpoints_sum[1][2], 0.f),
                        Vec4(endpoints_sum[2][0], endpoints_sum[2][1], endpoints_sum[2][2], 0.f),
                        Vec4(0.0f, 0.f, 0.f, 1.f));

        Vec3 b = Vec3(-endpoints_sum[3][0], -endpoints_sum[3][1], -endpoints_sum[3][2]);  // computed by extracting minus the upper-right 3x1 column from the same matrix

        Vec3 x_ = (e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 2;
        if (fabs(A.det()) > 1e-4)
            x_ = A.inverse() * b;

        this->optimal = x_;
        this->edge = e;

        Vec4 inter = endpoints_sum * Vec4(this->optimal[0], this->optimal[1], this->optimal[2], 1.f);
        this->cost = dot(inter, Vec4(this->optimal[0], this->optimal[1], this->optimal[2], 1.f));
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // TODO: check if the mesh only contains triangles

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    // Compute quadrics for each face by simply writing the plane equation for that face in homogeneous coordinates, 
    // and building the corresponding quadric matrix using outer(). This matrix should be stored in the yet-unmentioned
    // dictionary face_quadrics.

    // Compute an initial quadric for each vertex by adding up the quadrics at all the faces touching that vertex. 
    // This matrix should be stored in vertex_quadrics. (Note that these quadrics must be updated as edges are collapsed.)

    // For each edge, create an Edge_Record, insert it into the edge_records dictionary, and add it to one global 
    // PQueue<Edge_Record> queue.

    // Until a target number of triangles is reached, collapse the best/cheapest edge (as determined by the priority queue) 
    // and set the quadric at the new vertex to the sum of the quadrics at the endpoints of the original edge. 
    // You will also have to update the cost of any edge connected to this vertex.

    // faces
    size_t face_count = 0;
    for (auto f = faces_begin(); f != faces_end(); f++) {
        face_count += 1;
        auto normal = face_normal(f);
        float d = - dot(normal, f->halfedge()->vertex()->pos);
        face_quadrics[f] = outer(Vec4(normal, d), Vec4(normal, d));
    }

    // vertices
    for (auto v = vertices_begin(); v != vertices_end(); v++) {
        Mat4 sum = Mat4::Zero;
        Halfedge_Mesh::HalfedgeRef h = v->halfedge();
        do {
            sum += face_quadrics[h->face()];
            h = h->twin()->next();
        } while (h != v->halfedge());
        vertex_quadrics[v] = sum;
    }

    // edges
    for (auto e = edges_begin(); e != edges_end(); e++) {
        Edge_Record record = Edge_Record(vertex_quadrics, e);
        edge_records[e] = record;
        edge_queue.insert(record);
        printf("cost %f\n", record.cost);
    }

    size_t iteration = 0;

    size_t count = face_count;

    printf("map size: %zu\n", edge_records.size());
    printf("edge_queue size: %zu\n", edge_queue.size());

    // TODO: preserve manifoldness
    // bool stop = true;
    while (count > std::max(face_count / 4, (size_t)4) && edge_queue.size() > 0) {

        std::vector<EdgeRef> popped_edges;
        EdgeRef cheapest_e = edge_queue.top().edge;
        while(!can_collapse_edge(cheapest_e)) {
            edge_queue.pop();
            popped_edges.push_back(cheapest_e);
            cheapest_e = edge_queue.top().edge;
        }
        for(EdgeRef e : popped_edges) {
            Edge_Record r = edge_records[e];
            edge_queue.insert(r);
        }

        // Get the cheapest edge from the queue.
        EdgeRef best_edge = cheapest_e; // edge_queue.top().edge;
        auto opt_point = edge_records[best_edge].optimal;

        printf("queue size: %zu, edge record size: %zu\n", edge_queue.size(), edge_records.size());
        printf("picking edge %d\n", best_edge->id());

        // Remove the cheapest edge from the queue by calling pop().
        edge_queue.pop();

        // Compute the new quadric by summing the quadrics at its two endpoints.
        Mat4 new_quadrics = vertex_quadrics[best_edge->halfedge()->vertex()] + vertex_quadrics[best_edge->halfedge()->twin()->vertex()];

        // Remove any edge touching either of its endpoints from the queue.
        Halfedge_Mesh::HalfedgeRef h = best_edge->halfedge();
        do {
            edge_queue.remove(Edge_Record(vertex_quadrics, h->edge()));
            edge_records.erase(h->edge());
            h = h->twin()->next();
        } while (h != best_edge->halfedge());

        Halfedge_Mesh::HalfedgeRef h2 = best_edge->halfedge()->twin();
        do {
            edge_queue.remove(Edge_Record(vertex_quadrics, h2->edge()));
            edge_records.erase(h2->edge());
            h2 = h2->twin()->next();
        } while (h2 != best_edge->halfedge()->twin());

        // Collapse the edge.
        auto new_vertex = collapse_edge_erase(best_edge);
        if (!new_vertex.has_value())
            continue;

        new_vertex.value()->pos = opt_point;

        // Set the quadric of the new vertex to the quadric computed in Step 3.
        vertex_quadrics[new_vertex.value()] = new_quadrics;

        // Insert any edge touching the new vertex into the queue, creating new edge records for each of them.
        Halfedge_Mesh::HalfedgeRef h_ = new_vertex.value()->halfedge();
        do {
            auto new_record = Edge_Record(vertex_quadrics, h_->edge());
            edge_queue.insert(new_record);
            edge_records[h_->edge()] = new_record;
            h_ = h_->twin()->next();
        } while (h_ != new_vertex.value()->halfedge());

        iteration += 1;
        count -= 2;
    }

    printf("total iterations %zu", iteration);
    printf("total count left face count: %zu\n", count);

    return true;
}
