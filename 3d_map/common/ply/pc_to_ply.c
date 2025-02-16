/*

Sample code showing how to read and write PLY polygon files.

Greg Turk, March 1994

*/

#include <stdio.h>
#include <math.h>
#include <ply.h>
#include <pc_to_ply.h>

/* user's vertex and face definitions for a polygonal object */

typedef struct Vertex {
  float x,y,z;             /* the usual 3-space position of a vertex */
} Vertex;

/* information needed to describe the user's data to the PLY routines */

char *elem_names[] = { /* list of the kinds of elements in the user's object */
  "vertex",
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,z), 0, 0, 0, 0},
};



/******************************************************************************
Write out a PLY file.
******************************************************************************/

int32_t write_point_cloud_to_ply(char *file_name, int32_t num_points, point3f_t *pointers)
{
  int i,j;
  PlyFile *ply;
  int nelems;
  char **elist;
  int file_type;
  float version;
  int nverts = num_points;


  /* open either a binary or ascii PLY file for writing */
  /* (the file will be called "test.ply" because the routines */
  /*  enforce the .ply filename extension) */

#if 1
  ply = ply_open_for_writing(file_name, 1, elem_names, PLY_ASCII, &version);
#else
  ply = ply_open_for_writing("test", 1, elem_names, PLY_BINARY_BE, &version);
#endif

  /* describe what properties go into the vertex and face elements */

  ply_element_count (ply, "vertex", nverts);
  ply_describe_property (ply, "vertex", &vert_props[0]);
  ply_describe_property (ply, "vertex", &vert_props[1]);
  ply_describe_property (ply, "vertex", &vert_props[2]);


  /* write a comment and an object information field */
  ply_put_comment (ply, "author: MX");
  ply_put_obj_info (ply, "random information");

  /* we have described exactly what we will put in the file, so */
  /* we are now done with the header info */
  ply_header_complete (ply);

  /* set up and write the vertex elements */
  ply_put_element_setup (ply, "vertex");
  for (i = 0; i < nverts; i++)
    ply_put_element (ply, (void *) &pointers[i]);

  /* close the PLY file */
  ply_close (ply);
  return 0;
}
