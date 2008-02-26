// container to put the triangles in.
vector< HAPI::Bounds::Triangle > triangles;

// start collecting OpenGL primitives
FeedbackBufferCollector::startCollecting();

// draw your objects
draw();

// stop collecting and transfer the rendered triangles to the triangles vector.
FeedbackBufferCollector::endCollecting( triangles );
