FROM alpine:3.7 as builder

RUN apk update && \
    apk --no-cache add \
        ca-certificates \
        cmake \
        g++ \
        make
ADD . /opt/sources
WORKDIR /opt/sources
RUN cd /opt/sources && \
	mkdir build && \
    cd build && \
    cmake .. && \
    make helloworld && cp helloworld /tmp


#Deploy
FROM alpine:3.7
RUN apk update && \
    apk --no-cache add \
    libstdc++ libgcc && \
    mkdir /opt
WORKDIR /opt
COPY --from=builder /tmp/helloworld .

