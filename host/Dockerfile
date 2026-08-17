# Runtime image for the prebuilt host binaries.
# The binaries are built from the private optimal_solver source tree
# (see stage_binaries.sh) and copied into ./bin before building this image.
FROM ubuntu:22.04

RUN apt-get update \
 && apt-get install -y --no-install-recommends libgomp1 ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/optimal-solver
COPY bin/ bin/
COPY run_freeze.sh .

# Bind-mount an output directory: docker run -v "$PWD/output:/opt/optimal-solver/output" ...
RUN mkdir -p output
ENTRYPOINT ["/opt/optimal-solver/run_freeze.sh"]
