
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

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

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
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

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    // TODO: handle parallel edge collapse issue
    VertexRef v1 = e->halfedge()->vertex();
    VertexRef v2 = e->halfedge()->twin()->vertex();
    VertexRef v3 = new_vertex();
    v3->pos = (v1->pos + v2->pos) / 2;

    // change the half edges from v1 and v2
    auto edges_v1 = get_all_half_edges_of_vertex(v1);
    auto edges_v2 = get_all_half_edges_of_vertex(v2);
    std::vector<Halfedge_Mesh::HalfedgeRef> all_half_edges;
    all_half_edges.reserve(edges_v1.size() + edges_v2.size()); // preallocate memory
    all_half_edges.insert(all_half_edges.end(), edges_v1.begin(), edges_v1.end());
    all_half_edges.insert(all_half_edges.end(), edges_v2.begin(), edges_v2.end());

    for (auto halfedge: all_half_edges)
        halfedge->vertex() = v3;
    v3->halfedge() = all_half_edges[0];

    // collapse overlapping edges

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

    // ---- Existing elements ----
    HalfedgeRef h0 = e->halfedge(), h1 = h0->next(), h2 = h1->next();           // triangle 1
    HalfedgeRef h3 = h0->twin(), h4 = h3->next(), h5 = h4->next();              // triangle 2
    HalfedgeRef h6 = h1->twin(), h7 = h2->twin(), h8 = h4->twin(), h9 = h5->twin(); // outside

    VertexRef v0 = h0->vertex(), v1 = h3->vertex(), v2 = h8->vertex(), v3 = h6->vertex();

    EdgeRef e1 = h5->edge(), e2 = h4->edge(), e3 = h2->edge(), e4 = h1->edge();

    // ---- ALLOCATION ----
    HalfedgeRef h10 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h11 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h12 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h13 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h14 = Halfedge_Mesh::new_halfedge();
    HalfedgeRef h15 = Halfedge_Mesh::new_halfedge();

    FaceRef f2 = Halfedge_Mesh::new_face(), f3 = Halfedge_Mesh::new_face();

    VertexRef v4 = Halfedge_Mesh::new_vertex();

    EdgeRef e5 = Halfedge_Mesh::new_edge(), e6 = Halfedge_Mesh::new_edge(), e7 = Halfedge_Mesh::new_edge();

    // ---- Assignments ----
    // HalfedgeRef next, HalfedgeRef twin, VertexRef vertex, EdgeRef edge, FaceRef face
    h0->set_neighbors(h1,   h3,  v4, e,  f0); //
    h1->set_neighbors(h10,  h6,  v1, e4, f0);
    h2->set_neighbors(h12,  h7,  v3, e3, f2);
    h3->set_neighbors(h11,  h0,  v1, e,  f1);
    h4->set_neighbors(h14,  h8,  v0, e2, f3);
    h5->set_neighbors(h3,   h9,  v2, e1, f1);
    h6->set_neighbors(h9,   h1,  v3, e4, h6->face());
    h7->set_neighbors(h6,   h2,  v0, e3, h7->face());
    h8->set_neighbors(h7,   h4,  v2, e2, h8->face());
    h9->set_neighbors(h8,   h5,  v1, e1, h9->face());
    h10->set_neighbors(h0,  h15, v3, e5, f0);
    h11->set_neighbors(h5,  h14, v4, e6, f1); //
    h12->set_neighbors(h15, h13, v0, e7, f2);
    h13->set_neighbors(h4,  h12, v4, e7, f3); //
    h14->set_neighbors(h13, h11, v2, e6, f3);
    h15->set_neighbors(h2,  h10, v4, e5, f2); //

    // edges
    e->halfedge() = h0;
    e1->halfedge() = h9;
    e2->halfedge() = h8;
    e3->halfedge() = h7;
    e4->halfedge() = h6;
    e5->halfedge() = h10;
    e6->halfedge() = h11;
    e7->halfedge() = h12;

    // faces
    f0->halfedge() = h0;
    f1->halfedge() = h11;
    f2->halfedge() = h12;
    f3->halfedge() = h13;

    // vertices
    v0->halfedge() = h12;
    v1->halfedge() = h3;
    v2->halfedge() = h14;
    v3->halfedge() = h10;
    v4->halfedge() = h11;
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

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
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

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
