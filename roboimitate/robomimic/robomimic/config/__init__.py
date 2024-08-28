from robomimic.robomimic.config.config import Config
from robomimic.robomimic.config.base_config import config_factory, get_all_registered_configs

# note: these imports are needed to register these classes in the global config registry
from robomimic.robomimic.config.bc_config import BCConfig
from robomimic.robomimic.config.bcq_config import BCQConfig
from robomimic.robomimic.config.cql_config import CQLConfig
from robomimic.robomimic.config.iql_config import IQLConfig
from robomimic.robomimic.config.gl_config import GLConfig
from robomimic.robomimic.config.hbc_config import HBCConfig
from robomimic.robomimic.config.iris_config import IRISConfig
from robomimic.robomimic.config.td3_bc_config import TD3_BCConfig
from robomimic.robomimic.config.diffusion_policy_config import DiffusionPolicyConfig
