// container to put the triangles in.
vector< HAPI::Collision::Triangle > triangles;

// start collecting OpenGL primitives
HAPI::FeedbackBufferCollector::startCollecting();

// draw your objects
draw();

// stop collecting and transfer the rendered triangles to the triangles vector.
HAPI::FeedbackBufferCollector::endCollecting( triangles );
