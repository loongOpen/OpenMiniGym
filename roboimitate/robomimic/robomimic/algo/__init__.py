from robomimic.robomimic.algo.algo import register_algo_factory_func, algo_name_to_factory_func, algo_factory, Algo, PolicyAlgo, ValueAlgo, PlannerAlgo, HierarchicalAlgo, RolloutPolicy

# note: these imports are needed to register these classes in the global algo registry
from robomimic.robomimic.algo.bc import BC, BC_Gaussian, BC_GMM, BC_VAE, BC_RNN, BC_RNN_GMM
from robomimic.robomimic.algo.bcq import BCQ, BCQ_GMM, BCQ_Distributional
from robomimic.robomimic.algo.cql import CQL
from robomimic.robomimic.algo.iql import IQL
from robomimic.robomimic.algo.gl import GL, GL_VAE, ValuePlanner
from robomimic.robomimic.algo.hbc import HBC
from robomimic.robomimic.algo.iris import IRIS
from robomimic.robomimic.algo.td3_bc import TD3_BC
from robomimic.robomimic.algo.diffusion_policy import DiffusionPolicyUNet
