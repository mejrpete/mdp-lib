FROM ubuntu:18.04 AS compiled

RUN apt-get update \
    && apt-get -y install bison flex make g++ time libtool libbdd-dev

WORKDIR /app

COPY ./source/ /app

RUN make -j 5 libmdp rddl

#####################

FROM compiled AS final

ENV problem "gfb_inst_mdp__1"
ENV solver "laostar"
ENV heuristic "hmin"
ENV verbosity 100
ENV nsims 1
ENV interactive ""

CMD ["bash", "-c", "./testrddl.out --problemdir=/files/ --instance=${problem} --algorithm=${solver} --heuristic=${hmin} --nsims=${nsims} --v=${verbosity} ${interactive:+--interactive}"]