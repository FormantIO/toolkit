from typing import Callable, Dict, Optional, Type
import json
import jsonschema
import logging
from threading import Lock
from functools import reduce
import re

from formant.sdk.agent.v1.client import Client as AgentClient

logging.basicConfig()

JSONDecodeError = Exception  # type: Type[Exception]
try:
    JSONDecodeError = json.decoder.JSONDecodeError
except AttributeError:
    # python2 doesn't have JSON Decode Error
    pass


class JsonSchemaValidator:
    def __init__(
        self,
        client,  # type: AgentClient
        adapter_name,  # type: str
        update_adapter_config_callback,  # type: Callable[[Dict], None]
        validate=True,  # type: bool
        use_app_config=True,  # type: bool
        logger=None,  # type: Optional[logging.Logger]
        logger_level=logging.INFO,  # type: int
    ):
        self._lock = Lock()
        self._client = client
        self._adapter_name = adapter_name
        self._update_adapter_config_callback = update_adapter_config_callback
        self._use_app_config = use_app_config
        self._validate = validate
        if logger is None:
            logger = logging.getLogger(adapter_name)
            logger.setLevel(logger_level)
        self.logger = logger
        self._client.register_config_update_callback(self._update_adapter_config)
        if self._client.ignore_unavailable:
            self._update_adapter_config()

    def _update_adapter_config(self):
        # Consumer might not be threadsafe
        with self._lock:
            try:
                configs = [
                    self._get_config_from_adapter_configuration(),
                    self._get_config_from_json(),
                ]
                config = reduce(lambda s1, s2: s1 or s2, configs)
                if config is None:
                    raise Exception(
                        "Could not get configuration for '%s'" % self._adapter_name
                    )
                if self._use_app_config:
                    config = self._inject_app_config(config)
                if self._validate:
                    self._validate_schema(config)
                try:
                    self._update_adapter_config_callback(config)
                except Exception as e:
                    self.logger.error(
                        "Error calling update config callback %s" % str(e)
                    )
            except Exception as e:
                self.logger.warn("Failed to load config: %s" % str(e))
                self._client.create_event(
                    "%s configuration loading failed: %s."
                    % (self._adapter_name, str(e)),
                    notify=False,
                    severity="warning",
                )

    def _get_config_from_adapter_configuration(self):
        self.logger.info("Trying to get config from adapter config")
        try:
            adapters = self._client.get_agent_configuration().document.adapters
            for adapter in adapters:
                try:
                    config = json.loads(adapter.configuration)
                except:
                    continue
                if self._adapter_name in adapter.name:
                    self.logger.info("Got config from adapter config")
                    return config
        except Exception as e:
            self.logger.warn("Error getting config from adapter config: %s" % str(e))
        return None

    def _inject_app_config(self, config: Dict):
        """
        This function replaces all instances of
        {{key}} with `key` from app_config
        """
        config_string = json.dumps(config)
        pattern = r"\{\{(.+?)\}\}"
        matches = re.findall(pattern, config_string)
        for match in matches:
            val = self._client.get_app_config(match)
            config_string = config_string.replace("{{%s}}" % match, val)
        return json.loads(config_string)

    def _get_config_from_json(self):
        # Try to get config from config.json
        self.logger.info("Trying to get config from config.json file")
        try:
            with open("config.json") as f:
                config = json.loads(f.read())
                return config
        except Exception as e:
            self.logger.info("Error getting config from config.json: %s" % str(e))
        return None

    def _validate_schema(self, config_blob):
        # Validate configuration based on schema
        with open("config_schema.json") as f:
            try:
                self.config_schema = json.load(f)
                self.logger.info("Loaded config schema from config_schema.json file")
            except JSONDecodeError as e:
                self.logger.warn(
                    "Could not load config schema. Is the file valid json?"
                )
                raise Exception("config schema error: %s" % str(e))
        self.logger.info("Validating config...")
        try:
            jsonschema.validate(config_blob, self.config_schema)
            self.logger.info("Validation succeeded")
        except (
            jsonschema.ValidationError,
            jsonschema.SchemaError,
            jsonschema.RefResolutionError,
        ) as e:
            self.logger.warn("Validation failed %s: %s", type(e).__name__, str(e))
        except Exception as e:
            self.logger.warn(
                "Validation failed with unkown error %s: %s", type(e).__name__, str(e)
            )
