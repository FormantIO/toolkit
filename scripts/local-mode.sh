#!/usr/bin/env bash

function show_help() {
    echo "Usage: $0 [--clean|--shell]"
    echo "Setup/Starts a local InfluxDB container for Formant Local Mode"
    echo
    echo "Options:"
    echo "  --clean    Clean up the local InfluxDB container and remove data directory"
    echo "  --shell    Print the environment variables to be set in the agent.bashrc after the container is started"
    echo
    echo "Environment Variables:"
    echo "  INFLUX_PORT              InfluxDB port (default: 8086)"
    echo "  INFLUX_USERNAME          InfluxDB username (default: admin)"
    echo "  INFLUX_PASSWORD          InfluxDB password (default: admin123)"
    echo "  INFLUX_ORGANIZATION      InfluxDB organization (default: formant.local)"
    echo "  INFLUX_DATAPOINTS_BUCKET InfluxDB datapoints bucket (default: datapoints)"
    echo "  INFLUX_EVENTS_BUCKET     InfluxDB events bucket (default: events)"
    echo "  INFLUX_RETENTION_POLICY  InfluxDB retention policy (default: 30d)"
}

function do_clean() {
    echo "Cleaning up the local influxdb container..."
    if docker ps -a | grep -q $CONTAINER_NAME; then
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    fi
    rm -rf $INFLUX_DATA_DIR
}

CLEAN=false
SHELL=false

# Parse command line arguments
if [[ $# -eq 0 ]]; then
    SHELL=true
else
    while [[ $# -gt 0 ]]
    do
        key="$1"

        case $key in
            -h|--help|help)
                show_help
                exit 0
                ;;
            --clean)
                CLEAN=true
                shift
                ;;
            --shell)
                SHELL=true
                shift
                ;;
            *)
                echo "Unknown option $1"
                show_help
                exit 1
                ;;
        esac
    done
fi

## Set the environment variables
ENV=${ENV:-prod}
INFLUX_DATA_DIR=~/formant/$ENV/influxdb

INFLUX_PORT=${INFLUX_PORT:-8086}
INFLUX_USERNAME=${INFLUX_USERNAME:-admin}
INFLUX_PASSWORD=${INFLUX_PASSWORD:-admin123}
INFLUX_ORGANIZATION=${INFLUX_ORGANIZATION:-formant.local}

INFLUX_DATAPOINTS_BUCKET=${INFLUX_DATAPOINTS_BUCKET:-datapoints}
INFLUX_EVENTS_BUCKET=${INFLUX_EVENTS_BUCKET:-events}
INFLUX_RETENTION_POLICY=${INFLUX_RETENTION_POLICY:-30d}

CONTAINER_NAME=influxdb2.$ENV

mkdir -p $INFLUX_DATA_DIR/

if $CLEAN; then
    do_clean
    exit 0
fi

## If it is not running, start it
if ! docker ps -a | grep -q $CONTAINER_NAME; then
    echo "Starting the local influxdb container..."
    docker run -d \
        --name $CONTAINER_NAME \
        --restart unless-stopped \
        -p $INFLUX_PORT:$INFLUX_PORT \
        -v "$INFLUX_DATA_DIR/data:/var/lib/influxdb2" \
        -v "$INFLUX_DATA_DIR/config:/etc/influxdb2" \
        influxdb:2

    sleep 1

    ## Setup InfluxDB
    echo "Setting up the local influxdb container..."

    docker exec $CONTAINER_NAME influx setup \
    --username $INFLUX_USERNAME \
    --password $INFLUX_PASSWORD \
    --org $INFLUX_ORGANIZATION \
    --bucket $INFLUX_DATAPOINTS_BUCKET \
    --retention ${INFLUX_RETENTION_POLICY} \
    --force
    # Check for error and exit
    err=$?
    if [[ $err -ne 0 ]]; then
        echo "ERROR: Failed to setup influxdb: $err"
        do_clean
        exit 1
    fi
    for bucket in ${INFLUX_EVENTS_BUCKET}; do
        echo "Creating bucket $bucket"
        docker exec $CONTAINER_NAME influx bucket create \
            --name $bucket \
            --org ${INFLUX_ORGANIZATION} \
            --retention ${INFLUX_RETENTION_POLICY}
    done

    ADMIN_INFO=

    while [[ -z $ADMIN_INFO ]]
    do
        ADMIN_INFO=$(docker exec $CONTAINER_NAME influx auth list)
    done

    INFLUXDB_TOKEN=$(docker exec $CONTAINER_NAME influx auth list | grep ${INFLUX_USERNAME} | awk '{ print $4 }')

    echo "Got token ${INFLUXDB_TOKEN} from influxdb"

    if $SHELL; then
        echo "Add the following to the agent.bashrc:"
        printf "export FORMANT_INFLUX_TOKEN=${INFLUXDB_TOKEN}\n"
        printf "export FORMANT_INFLUX_ORG=${INFLUX_ORGANIZATION}\n"
        printf "export FORMANT_LOCAL=true\n"
        printf "export FORMANT_INFLUX_URL=http://localhost:${INFLUX_PORT}\n\n"
    else
        FORMANT_INFLUX_TOKEN=${INFLUXDB_TOKEN} \
        FORMANT_INFLUX_ORG=${INFLUX_ORGANIZATION} \
        FORMANT_LOCAL=true \
        FORMANT_INFLUX_URL=http://localhost:${INFLUX_PORT}
    fi
else
    docker start $CONTAINER_NAME
fi
