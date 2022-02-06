COORDS trilateration(&anchor_coords,&anchor_distances)
{
// Passing through the original anchor node coordinates into our own structs to use
   COORDS_DIST p1
        p1.x = anchor_coords.x[0];
        p1.y = anchor_coords.y[0];
        p1.z = anchor_coords.z[0];
        p1.radius = anchor_distances[0];
   COORDS_DIST p2
        p2.x = anchor_coords.x[1];
        p2.y = anchor_coords.y[1];
        p2.z = anchor_coords.z[1];
        p2.radius = anchor_distances[1];

   COORDS_DIST p3
        p3.x = anchor_coords.x[2];
        p3.y = anchor_coords.y[2];
        p3.z = anchor_coords.z[2];
        p3.radius = anchor_distances[2];

 // Redefined anchor node coordinates with p1 (the first anchor node) as the origin
   COORDS p1a
        p1a.x = p1.x[0]-p1.x[0];
        p1a.y = p1.y[0]-p1.y[0];
        p1a.z = p1.z[0]-p1.z[0];
   COORDS p2a
        p2a.x = p2.x[0]-p1.x[0];
        p2a.y = p2.y[0]-p1.y[0];
        p2a.z = p2.z[0]-p1.z[0];
   COORDS p3a
        p3a.x = p3.x[0]-p1.x[0];
        p3a.y = p3.y[0]-p1.y[0];
        p3a.z = p3.z[0]-p1.z[0];

    // Probably need to convert to pointers
    d = sqrt(p2a[0]^2 + p2a[1]^2 + p2a[2]^2);
    ex = p2a/d;
    i = dotProduct(ex, p3a);

}