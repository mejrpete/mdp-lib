FROM ubuntu:18.04 AS compiled

RUN apt-get update \
    && apt-get -y install bison flex make g++ time libtool libbdd-dev

WORKDIR /app

COPY ./source/ /app

RUN make minigpt libmdp ppddl rddl
RUN make test

#####################

FROM compiled AS final

ENV dir /files/
ENV problem academic-advising_inst_mdp__01

CMD cd scripts && ./run_testrddl.sh ${dir} ${problem}
