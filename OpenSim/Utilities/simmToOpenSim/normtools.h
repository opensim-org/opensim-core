/*******************************************************************************

   NORMTOOLS.H

   Author: Peter Loan

   Date: 12-SEP-96

   Copyright (c) 1996 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/

void norm(PolyhedronStruct* ph_in, NormOptions* opt, int* num_output,
      PolyhedronStruct** ph_out);
PolyhedronStruct* combine_polyhedra(PolyhedronStruct** phs, int num_phs);
ReturnCode check_polyhedron(PolyhedronStruct* ph);
void preread_init_polygon(PolygonStruct* p);
void preread_init_vertex(VertexStruct* v, int index);
void preread_init_polyhedron(PolyhedronStruct* ph);
void postread_init_polyhedron(PolyhedronStruct* ph, SBoolean full_init);
PolyhedronStruct* clone_polyhedron(PolyhedronStruct* from);
void copy_polyhedron(PolyhedronStruct* from, PolyhedronStruct* to);
void copy_vertex(VertexStruct* from, VertexStruct* to);
void copy_vertex_lite(VertexStruct* from, VertexStruct* to);
void copy_pintersegmentstruct(PInterSegmentStruct* from,
                  PInterSegmentStruct* to);
void copy_pintersegmentliststruct(PInterSegmentListStruct* from,
                  PInterSegmentListStruct* to);
void copy_polygon(PolygonStruct* from, PolygonStruct* to);
void copy_polygon_lite(PolygonStruct* from, PolygonStruct* to);
void calc_polygon_bounding_cube(PolyhedronStruct* ph, PolygonStruct* p);
void calc_polyhedron_bounding_cube(PolyhedronStruct* ph);
void make_vert_poly_lists(PolyhedronStruct* ph);
int find_other_polygon(PolyhedronStruct* ph, int p1, int v1, int v2);
int subdivide_input_polyhedron(PolyhedronStruct* ip, PolyhedronStruct** phs);
SBoolean vertex_in_polygon(PolygonStruct* polygon, int v_num);
void find_vertex_normals(PolyhedronStruct* ph, NormOptions* opt);
void make_polygons_consistent(PolyhedronStruct* ph, NormOptions* opt);
void calc_vertex_normals(PolyhedronStruct* ph, NormOptions* opt);
void order_all_vertex_lists(PolyhedronStruct* ph, int polygon);
void calc_polygon_normal(PolyhedronStruct* ph, PolygonStruct* p);
void match_vertex_orders(PolygonStruct* polygon1, PolygonStruct* polygon2,
             int vertex1, int vertex2);
void reverse_vertex_order(PolygonStruct* p);
void find_polygons_sharing_edge(PolyhedronStruct* ph, int vertex1, int vertex2,
                int* polygon1, int* polygon2);
VertexOrder polygon_contains_edge(PolygonStruct* p, int vertex1, int vertex2);
int find_top_vertex(VertexStruct* vertex, int num_vertices);
int find_shallowest_vertex(PolyhedronStruct* ph, int top_vertex);
VerticesFound get_adjacent_vertices(PolygonStruct* p, int seed_vertex,
                    int* previous_vertex, int* next_vertex);
void find_other_edge(PolyhedronStruct* ph, int poly, int start_vertex,
             int adj_vertex, double other_edge[]);
int compare_dist(PListStruct* item1, PListStruct* item2);
void find_zero_area_polygons(PolyhedronStruct* ph);
void remove_zero_area_polygons(PolyhedronStruct* ph);
void find_zero_area_nontriangle(PolyhedronStruct* ph, int pl);
void remove_zero_area_nontriangle(PolyhedronStruct* ph, int pl);
void remove_duplicate_vertices(PolyhedronStruct* ph, NormOptions* opt);
void add_polygons_to_vert_poly_list(VertexStruct* v,
                   int* polygons, int polygon_count);
void add_polygon_to_vert_poly_list(VertexStruct* v, int poly_num);
void remove_polygon_from_vert_poly_list(VertexStruct* v, int poly_num);
void replace_vertex_index(PolygonStruct* p, int old_index, int new_index);
void remove_degenerate_polygons(PolyhedronStruct* ph);
void remove_overlapping_polygons(PolyhedronStruct* ph);
int vertex_lists_identical(PolyhedronStruct* ph, int poly1, int poly2);
void remove_extra_polygons(PolyhedronStruct* ph);
int fix_polygon(PolyhedronStruct* ph, int pl);
void order_polygons(PolyhedronStruct* polyhedron, int order_format);
void swap_polygons(PolyhedronStruct* ph, int index1, int index2);
void order_vertices(PolyhedronStruct* polyhedron, int order_format);
void swap_vertex_indices(PolyhedronStruct* ph, int index1, int index2);
void fill_holes(PolyhedronStruct* ph, SBoolean fill_holes);
void check_edge_usage(PolyhedronStruct* ph);
void check_edge_lengths(PolyhedronStruct* ph, double max_edge_length);
void triangulate_polygons(PolyhedronStruct* ph, TriangulateOption triangulate);
void quarter_quad(PolyhedronStruct* ph, int poly_num);
void split_triangle(PolyhedronStruct* ph, int poly_num);
SBoolean polygon_very_concave(PolyhedronStruct* ph, int poly_num);
void print_internal_angles(PolyhedronStruct* ph, int pl);
void trisect_fiver(PolyhedronStruct* ph, int old_polygon);
void bisect_sixer(PolyhedronStruct* ph, int old_polygon);
int find_largest_angle(PolyhedronStruct* ph, int poly_num);
void change_vertex_indices(PolyhedronStruct* ph, int p_num, int v_index[], int num_v);
void split_large_polygon(PolyhedronStruct* ph, int poly_num, SBoolean realloc_vertices,
             SBoolean realloc_polygons);
void reorient_polyhedron(PolyhedronStruct* ph, double rot_mat[][3]);
void unorient_polyhedron(PolyhedronStruct* ph, double rot_mat[][3]);
int make_new_vertex(PolyhedronStruct* ph, double pt[], int ph_num,
            SBoolean check_existing, SBoolean realloc_array);
int make_new_polygon(PolyhedronStruct* ph, int num_vertices, int v_index[],
             int ph_num, double normal[], double d, SBoolean realloc_array,
             SBoolean recalc_normal);
void add_poly_to_vert_poly_lists(PolyhedronStruct* ph, PolygonStruct* p, int poly_num);
void convexify_polygons(PolyhedronStruct* ph, NormOptions* opt);
int polygon_is_concave(PolyhedronStruct* ph, int poly);
void split_edge(PolyhedronStruct* bone, int poly_num, int edge_num);
int find_nth_polygon(PolyhedronStruct* bone, int vertex_num, int* n);
void add_vertex(PolyhedronStruct* ph, int poly, int e1, int e2, int v_new);
void add_vert_to_poly(PolyhedronStruct* ph, int poly, int vert1, int vert2, int v_new);
void bisect_polygon(PolyhedronStruct* ph, int poly, int v1, int v2);
int find_bisection_point(PolyhedronStruct* ph, int poly, int v1, double p_bisect[]);
int polygon_contains_edges(PolygonStruct* p, int v1, int v2, int v3);
void compress_polyhedron(PolyhedronStruct* ph);
void throw_out_vertex(PolyhedronStruct* ph, int v);
void throw_out_polygon(PolyhedronStruct* ph, int p);
void print_polyhedron(PolyhedronStruct* ph, char pname[]);
void print_vertex(VertexStruct* v);
void print_polygon(PolygonStruct* p);
void print_polyhedron_simple(PolyhedronStruct* ph, char pname[]);
void free_polyhedron(PolyhedronStruct* ph, SBoolean free_ph, ModelStruct* ms);
void find_bounding_cube(PolyhedronStruct* polyhedron, BoundingCube* bc);
void unscale_bones(ModelStruct* model);

#if ! NORM && ! ENGINE && ! OPENSMAC
void delete_polyhedron_display_list(PolyhedronStruct* ph, ModelStruct* ms);
#endif
