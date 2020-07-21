FROM ubuntu:18.04 AS base

RUN apt-get update \
    && apt-get -y install apt-utils \
    && apt -y install mercurial g++ g++-multilib make cmake bison flex python automake autoconf libtool libbdd-dev

#####################


FROM base AS parserlayer 

WORKDIR /parser

COPY ./source/rddl_parser /parser/rddl_parser
COPY ./source/build_1 /parser/
RUN ./build_1

FROM parserlayer AS plannerlayer
COPY ./source/search /parser/search
COPY ./source/build_2 /parser/
RUN ./build_2

FROM plannerlayer AS final

ENV filesdir /files
ENV inputfile input.rddl

## Call your planner
CMD ["bash", "-c", "cd search && ./parser ${filesdir} ${inputfile}"]