FROM ubuntu:14.04 AS compiled

RUN apt-get update \
    && apt-get -y install bison flex make g++ time

WORKDIR /app

COPY ./source/ /app

RUN make libmdp ppddl
RUN make test

#####################

FROM compiled AS final

ENV dir ../data/ppddl/ippc2006
ENV domain elevators
ENV instnum 01

CMD cd scripts && ./run_testppddl_laostar.sh ${dir} ${domain} ${instnum}
