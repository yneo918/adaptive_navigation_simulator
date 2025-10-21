import numpy as np
import math
import sympy as sp 
from enum import Enum
from typing import Optional, List, Tuple, Union
from abc import ABC, abstractmethod

class ClusterConfig(Enum): 
    """Enumeration of available cluster configurations."""
    TRICEN = "TriangleatCentroid" 
    TRILEAD = "TriangleatLeader" 
    PENLEAD = "PentagonatLeader" 
    
    def __str__(self):
        return self.value

class ClusterError(Exception):
    """Custom exception for cluster-related errors."""
    pass

class ControlMode(Enum):
    """Control modes for cluster operation."""
    POSITION = "POS"
    VELOCITY = "VEL"

# Constants
ROVER_DOF = 3
DEFAULT_GAIN = 1.0

class ClusterConfigurationBase(ABC):
    """Abstract base class for cluster configurations."""
    
    @abstractmethod
    def setup_kinematics(self) -> Tuple[sp.Matrix, sp.Matrix, sp.Matrix, sp.Matrix, List[int]]:
        """
        Setup kinematic equations for the cluster configuration.
        
        Returns:
            Tuple containing (FKine, IKine, Jacobian, JacobianInv, angle_indices)
        """
        pass
    
    @staticmethod
    def _create_lambdified_functions(fkine: sp.Matrix, ikine: sp.Matrix, 
                                   jacob: sp.Matrix, jacob_inv: sp.Matrix,
                                   r_symbols: List[sp.Symbol], c_symbols: List[sp.Symbol]):
        """Create lambdified functions from symbolic expressions."""
        return {
            'fkine': sp.lambdify(r_symbols, fkine, 'numpy'),
            'ikine': sp.lambdify(c_symbols, ikine, 'numpy'),
            'jacobian': sp.lambdify(r_symbols, jacob, 'numpy'),
            'jacobian_inv': sp.lambdify(c_symbols, jacob_inv, 'numpy')
        }

class TriangleCentroidConfig(ClusterConfigurationBase):
    """Configuration for triangle at centroid with 3 robots."""
    
    def setup_kinematics(self) -> Tuple[sp.Matrix, sp.Matrix, sp.Matrix, sp.Matrix, List[int]]:
        r_sym = sp.symbols('r0:9')  # Robot state variables
        c_sym = sp.symbols('c0:9')  # Cluster state variables

        # Forward kinematics: robot space -> cluster space
        x_c = (r_sym[0] + r_sym[3] + r_sym[6]) / 3
        y_c = (r_sym[1] + r_sym[4] + r_sym[7]) / 3
        theta_c = sp.atan2(
            2/3 * r_sym[0] - 1/3 * (r_sym[3] + r_sym[6]), 
            2/3 * r_sym[1] - 1/3 * (r_sym[4] + r_sym[7])
        )
        
        # Robot orientations relative to cluster
        phi1 = r_sym[2] + theta_c
        phi2 = r_sym[5] + theta_c
        phi3 = r_sym[8] + theta_c
        
        # Triangle side lengths and angle
        p = sp.sqrt((r_sym[0] - r_sym[3])**2 + (r_sym[1] - r_sym[4])**2)
        q = sp.sqrt((r_sym[6] - r_sym[0])**2 + (r_sym[1] - r_sym[7])**2)
        B = sp.acos((p**2 + q**2 - (r_sym[6] - r_sym[3])**2 - (r_sym[7] - r_sym[4])**2) / (2*p*q))

        # Inverse kinematics: cluster space -> robot space
        r = sp.sqrt((c_sym[7] + c_sym[6]*sp.cos(c_sym[8]))**2 + (c_sym[6]*sp.sin(c_sym[8]))**2)
        
        # Robot positions
        x_1 = c_sym[0] + r/3 * sp.sin(c_sym[2])
        y_1 = c_sym[1] + r/3 * sp.cos(c_sym[2])
        theta_1 = c_sym[3] - c_sym[2]
        
        x_2 = c_sym[0] + r/3 * sp.sin(c_sym[2]) - c_sym[6] * sp.sin(c_sym[8]/2 + c_sym[2])
        y_2 = c_sym[1] + r/3 * sp.cos(c_sym[2]) - c_sym[6] * sp.cos(c_sym[8]/2 + c_sym[2])
        theta_2 = c_sym[4] - c_sym[2]
        
        x_3 = c_sym[0] + r/3 * sp.sin(c_sym[2]) + c_sym[7] * sp.sin(c_sym[8]/2 - c_sym[2])
        y_3 = c_sym[1] + r/3 * sp.cos(c_sym[2]) - c_sym[7] * sp.cos(c_sym[8]/2 - c_sym[2])
        theta_3 = c_sym[5] - c_sym[2]
        
        fkine = sp.Matrix([[x_c], [y_c], [theta_c], [phi1], [phi2], [phi3], [p], [q], [B]])
        ikine = sp.Matrix([[x_1], [y_1], [theta_1], [x_2], [y_2], [theta_2], [x_3], [y_3], [theta_3]])
        jacobian = fkine.jacobian(r_sym)
        jacobian_inv = ikine.jacobian(c_sym)
        
        angle_indices = [2, 3, 4, 5, 8]
        
        return fkine, ikine, jacobian, jacobian_inv, angle_indices

class TriangleLeaderConfig(ClusterConfigurationBase):
    """Configuration for triangle at centroid with 3 robots."""
    
    def setup_kinematics(self) -> Tuple[sp.Matrix, sp.Matrix, sp.Matrix, sp.Matrix, List[int]]:
        r_sym = sp.symbols('r0:9')  # Robot state variables
        c_sym = sp.symbols('c0:9')  # Cluster state variables

        # Forward kinematics: robot space -> cluster space
        x_c = r_sym[0]
        y_c = r_sym[1]
        theta_c = r_sym[2]
        
        # Robot orientations relative to cluster
        phi2 = r_sym[5] + theta_c
        phi3 = r_sym[8] + theta_c
        
        # Triangle side lengths and angle
        d2 = sp.sqrt((r_sym[3] - r_sym[0])**2 + (r_sym[4] - r_sym[1])**2)
        d3 = sp.sqrt((r_sym[6] - r_sym[0])**2 + (r_sym[7] - r_sym[1])**2)
        B2 = sp.atan2(r_sym[4] - r_sym[1], r_sym[3] - r_sym[0]) - theta_c
        B3 = sp.atan2(r_sym[7] - r_sym[1], r_sym[6] - r_sym[0]) - theta_c
        
        # Robot positions
        x_1 = c_sym[0]
        y_1 = c_sym[1]
        theta_1 = c_sym[3]
        
        x_2 = c_sym[0] + d2 * sp.cos(B2 + c_sym[2])
        y_2 = c_sym[1] + d2 * sp.sin(B2 + c_sym[2])
        theta_2 = c_sym[4] + c_sym[2]

        x_3 = c_sym[0] + d3 * sp.cos(B3 + c_sym[2])
        y_3 = c_sym[1] + d3 * sp.sin(B3 + c_sym[2])
        theta_3 = c_sym[5] + c_sym[2]
        
        fkine = sp.Matrix([[x_c], [y_c], [theta_c], [phi2], [phi3], [d2], [d3], [B2], [B3]])
        ikine = sp.Matrix([[x_1], [y_1], [theta_1], [x_2], [y_2], [theta_2], [x_3], [y_3], [theta_3]])
        jacobian = fkine.jacobian(r_sym)
        jacobian_inv = ikine.jacobian(c_sym)
        
        angle_indices = [2, 3, 4, 5, 8]
        
        return fkine, ikine, jacobian, jacobian_inv, angle_indices

class PentagonLeaderConfig(ClusterConfigurationBase):
    """Configuration for Pentagon with leader robot (5 robots)."""
    
    def setup_kinematics(self) -> Tuple[sp.Matrix, sp.Matrix, sp.Matrix, sp.Matrix, List[int]]:
        r_sym = sp.symbols('r0:15')  # Robot state variables
        c_sym = sp.symbols('c0:15')  # Cluster state variables

        # Forward kinematics: robot space -> cluster space
        x_c = r_sym[0]  # Leader robot position
        y_c = r_sym[1]
        theta_c = sp.atan2(r_sym[4] - r_sym[1], r_sym[3] - r_sym[0])

        # Robot orientations relative to cluster
        phi = [r_sym[i*3 + 2] - theta_c for i in range(5)]
        
        # Distances from leader/reference points
        d = [0]  # Leader has zero distance from itself
        d.append(sp.sqrt((r_sym[0] - r_sym[3])**2 + (r_sym[1] - r_sym[4])**2))
        d.append(sp.sqrt((r_sym[6] - r_sym[0])**2 + (r_sym[7] - r_sym[1])**2))
        d.append(sp.sqrt((r_sym[3] - r_sym[9])**2 + (r_sym[4] - r_sym[10])**2))
        d.append(sp.sqrt((r_sym[6] - r_sym[12])**2 + (r_sym[7] - r_sym[13])**2))
        
        # Bearing angles
        beta = [0, 0]  # First two robots have zero bearing
        beta.append(sp.atan2(r_sym[7] - r_sym[1], r_sym[6] - r_sym[0]) - theta_c - sp.pi)
        # robot1 -> robot3
        beta.append(sp.atan2(r_sym[10] - r_sym[4], r_sym[9] - r_sym[3]) - theta_c - sp.pi / 2)
        # robot2 -> robot4
        beta.append(sp.atan2(r_sym[13] - r_sym[7], r_sym[12] - r_sym[6]) - theta_c - sp.pi / 2)

        # Inverse kinematics: cluster space -> robot space
        x = [c_sym[0]]
        y = [c_sym[1]]
        theta = [c_sym[2] + c_sym[3]]

        # Calculate remaining robot positions
        x.append(x[0] + c_sym[8] * sp.cos(c_sym[2]))
        y.append(y[0] + c_sym[8] * sp.sin(c_sym[2]))
        theta.append(c_sym[2] + c_sym[4])

        x.append(x[0] + c_sym[9] * sp.cos(c_sym[12] + c_sym[2] + sp.pi))
        y.append(y[0] + c_sym[9] * sp.sin(c_sym[12] + c_sym[2] + sp.pi))
        theta.append(c_sym[2] + c_sym[5])

        x.append(x[1] + c_sym[10] * sp.cos(c_sym[13] + c_sym[2] + sp.pi/2))
        y.append(y[1] + c_sym[10] * sp.sin(c_sym[13] + c_sym[2] + sp.pi/2))
        theta.append(c_sym[2] + c_sym[6])

        x.append(x[2] + c_sym[11] * sp.cos(c_sym[14] + c_sym[2] + sp.pi/2))
        y.append(y[2] + c_sym[11] * sp.sin(c_sym[14] + c_sym[2] + sp.pi/2))
        theta.append(c_sym[2] + c_sym[7])

        fkine = sp.Matrix([x_c, y_c, theta_c] + phi + d[1:] + beta[2:])
        ikine = sp.Matrix([val for triplet in zip(x, y, theta) for val in triplet])
        jacobian = fkine.jacobian(r_sym)
        jacobian_inv = ikine.jacobian(c_sym)
        
        angle_indices = [2, 3, 4, 5, 6, 7, 12, 13, 14]
        
        return fkine, ikine, jacobian, jacobian_inv, angle_indices

class Cluster:
    """
    Multi-robot cluster control system.
    
    Manages the state space variables and kinematic transforms for a cluster of robots
    in various geometric configurations. Supports position and velocity control modes.
    
    Args:
        num_robots: Number of robots in the cluster
        cluster_type: Geometric configuration of the cluster
        kp_gains: Proportional control gains (length: num_robots * DOF)
        kv_gains: Derivative control gains (length: num_robots * DOF)
        control_mode: Control mode ('POS' for position, 'VEL' for velocity)
    """
    
    # Configuration registry
    # To add new configurations write out the kinematic equations in a new class inheriting ClusterConfigurationBase and add the enum name here
    _CONFIG_REGISTRY = {
        (3, ClusterConfig.TRICEN): TriangleCentroidConfig,
        (3, ClusterConfig.TRILEAD): TriangleCentroidConfig,
        (5, ClusterConfig.PENLEAD): PentagonLeaderConfig,
    }
    
    def __init__(self, 
                 num_robots: int = 3, 
                 cluster_type: Union[ClusterConfig, str] = ClusterConfig.TRICEN,
                 kp_gains: Optional[List[float]] = None, 
                 kv_gains: Optional[List[float]] = None,
                 control_mode: Union[ControlMode, str] = ControlMode.POSITION):
        
        self.num_robots = self._validate_num_robots(num_robots)
        self.cluster_type = self._validate_cluster_type(cluster_type)
        self.control_mode = self._validate_control_mode(control_mode)
        
        # Initialize gain matrices
        self.kp, self.kv = self._initialize_gains(kp_gains, kv_gains)
        
        # Initialize cluster configuration
        self._setup_cluster_configuration()
    
    def _validate_num_robots(self, num_robots: int) -> int:
        """Validate number of robots."""
        if not isinstance(num_robots, int) or num_robots <= 0:
            raise ClusterError(f"Number of robots must be a positive integer, got {num_robots}")
        return num_robots
    
    def _validate_cluster_type(self, cluster_type: Union[ClusterConfig, str]) -> ClusterConfig:
        """Validate and convert cluster type."""
        if isinstance(cluster_type, str):
            try:
                return ClusterConfig(cluster_type)
            except ValueError:
                valid_types = [config.value for config in ClusterConfig]
                raise ClusterError(f"Invalid cluster type '{cluster_type}'. Valid types: {valid_types}")
        elif isinstance(cluster_type, ClusterConfig):
            return cluster_type
        else:
            raise ClusterError(f"Cluster type must be ClusterConfig enum or string, got {type(cluster_type)}")
    
    def _validate_control_mode(self, control_mode: Union[ControlMode, str]) -> ControlMode:
        """Validate and convert control mode."""
        if isinstance(control_mode, str):
            try:
                return ControlMode(control_mode)
            except ValueError:
                valid_modes = [mode.value for mode in ControlMode]
                raise ClusterError(f"Invalid control mode '{control_mode}'. Valid modes: {valid_modes}")
        elif isinstance(control_mode, ControlMode):
            return control_mode
        else:
            raise ClusterError(f"Control mode must be ControlMode enum or string, got {type(control_mode)}")
    
    def _initialize_gains(self, kp_gains: Optional[List[float]], 
                         kv_gains: Optional[List[float]]) -> Tuple[np.ndarray, np.ndarray]:
        """Initialize and validate control gain matrices."""
        expected_length = self.num_robots * ROVER_DOF
        
        if kp_gains is None:
            kp_gains = [DEFAULT_GAIN] * expected_length
        if kv_gains is None:
            kv_gains = [DEFAULT_GAIN] * expected_length
            
        if len(kp_gains) != expected_length:
            raise ClusterError(f"KP gains must have length {expected_length}, got {len(kp_gains)}")
        if len(kv_gains) != expected_length:
            raise ClusterError(f"KV gains must have length {expected_length}, got {len(kv_gains)}")
        
        return np.diag(kp_gains), np.diag(kv_gains)
    
    def _setup_cluster_configuration(self):
        """Setup cluster configuration based on number of robots and type."""
        config_key = (self.num_robots, self.cluster_type)
        
        if config_key not in self._CONFIG_REGISTRY:
            available_configs = list(self._CONFIG_REGISTRY.keys())
            raise ClusterError(
                f"Configuration {self.cluster_type.value} not implemented for {self.num_robots} robots. "
                f"Available configurations: {available_configs}"
            )
        
        config_class = self._CONFIG_REGISTRY[config_key]
        config_instance = config_class()
        
        # Setup kinematic equations
        fkine, ikine, jacobian, jacobian_inv, angle_indices = config_instance.setup_kinematics()
        
        # Create symbolic variables for lambdification
        r_symbols = sp.symbols(f'r0:{self.num_robots * ROVER_DOF}')
        c_symbols = sp.symbols(f'c0:{len(fkine)}')
        
        # Create lambdified functions
        functions = ClusterConfigurationBase._create_lambdified_functions(
            fkine, ikine, jacobian, jacobian_inv, r_symbols, c_symbols
        )
        
        # Store functions and matrices
        self.fkine_func = functions['fkine']
        self.ikine_func = functions['ikine']
        self.jacobian_func = functions['jacobian']
        self.jacobian_inv_func = functions['jacobian_inv']
        
        self.fkine_matrix = fkine
        self.ikine_matrix = ikine
        self.jacobian_matrix = jacobian
        self.jacobian_inv_matrix = jacobian_inv
        
        self.cluster_angle_indices = angle_indices
    
    def get_velocity_command(self, robot_positions: np.ndarray, robot_velocities: np.ndarray,
                           desired_cluster_state: np.ndarray, desired_cluster_velocity: np.ndarray
                           ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculate robot velocity commands based on desired cluster state.
        
        Args:
            robot_positions: Current robot positions (num_robots * DOF,)
            robot_velocities: Current robot velocities (num_robots * DOF,)
            desired_cluster_state: Desired cluster state variables
            desired_cluster_velocity: Desired cluster velocity
            
        Returns:
            Tuple of (cluster_velocity_command, robot_velocity_command, current_cluster_state)
        """
        current_cluster_state = self.get_cluster_position(robot_positions)
        current_cluster_velocity = self.calculate_cluster_velocity(robot_positions, robot_velocities)
        
        cluster_velocity_command = self._calculate_control_command(
            current_cluster_state, current_cluster_velocity,
            desired_cluster_state, desired_cluster_velocity
        )
        
        robot_velocity_command = np.dot(
            np.array(self.jacobian_inv_func(*current_cluster_state.flatten())), 
            cluster_velocity_command
        )
        
        return cluster_velocity_command, robot_velocity_command, current_cluster_state
    
    def get_cluster_position(self, robot_positions: np.ndarray) -> np.ndarray:
        """
        Calculate cluster state variables from robot positions.
        
        Args:
            robot_positions: Robot positions (num_robots * DOF,)
            
        Returns:
            Cluster state variables
        """
        cluster_state = np.array(self.fkine_func(*robot_positions.flatten())).astype(np.float64)
        
        # Wrap angles to [-π, π]
        for i in self.cluster_angle_indices:
            cluster_state[i, 0] = self._wrap_to_pi(cluster_state[i, 0])
            
        return cluster_state
    
    def calculate_cluster_velocity(self, robot_positions: np.ndarray, 
                                 robot_velocities: np.ndarray) -> np.ndarray:
        """
        Calculate cluster velocity from robot states.
        
        Args:
            robot_positions: Robot positions (num_robots * DOF,)
            robot_velocities: Robot velocities (num_robots * DOF,)
            
        Returns:
            Cluster velocity
        """
        jacobian = np.array(self.jacobian_func(*robot_positions.flatten())).astype(np.float64)
        return np.dot(jacobian, robot_velocities)
    
    def _calculate_control_command(self, current_cluster_state: np.ndarray,
                                 current_cluster_velocity: np.ndarray,
                                 desired_cluster_state: np.ndarray,
                                 desired_cluster_velocity: np.ndarray) -> np.ndarray:
        """Calculate control command based on control mode."""
        if self.control_mode == ControlMode.VELOCITY:
            for i in range(ROVER_DOF, len(desired_cluster_velocity)):
                desired_cluster_velocity[i, 0] = desired_cluster_state[i, 0] - current_cluster_state[i, 0]
            return desired_cluster_velocity
        
        elif self.control_mode == ControlMode.POSITION:
            state_error = desired_cluster_state - current_cluster_state
            
            # Wrap angular errors to [-π, π]
            for i in self.cluster_angle_indices:
                state_error[i, 0] = self._wrap_to_pi(state_error[i, 0])
            
            return np.dot(self.kp, state_error)
        
        else:
            raise ClusterError(f"Unknown control mode: {self.control_mode}")
    
    def get_desired_robot_positions(self, desired_cluster_state: np.ndarray) -> Optional[np.ndarray]:
        """
        Calculate desired robot positions from cluster state.
        
        Args:
            desired_cluster_state: Desired cluster state variables
            
        Returns:
            Desired robot positions or None if inverse kinematics unavailable
        """
        if self.ikine_func is None:
            return None
        
        return np.array(self.ikine_func(*desired_cluster_state.flatten())).astype(np.float64)
    
    def test_kinematic_transforms(self, robot_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Test forward and inverse kinematic transforms for consistency.
        
        Args:
            robot_positions: Initial robot positions
            
        Returns:
            Tuple of (cluster_state, recovered_robot_positions)
        """
        cluster_state = np.array(self.fkine_func(*robot_positions.flatten())).astype(np.float64)
        recovered_positions = np.array(self.ikine_func(*cluster_state.flatten())).astype(np.float64)
        return cluster_state, recovered_positions
    
    def export_symbolic_expressions(self, output_dir: str = 'cluster_expressions'):
        """
        Export symbolic kinematic expressions to text files.
        
        Args:
            output_dir: Directory to save expression files
        """
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        expressions = {
            'FKine.txt': self.fkine_matrix,
            'IKine.txt': self.ikine_matrix,
            'Jacobian.txt': self.jacobian_matrix,
            'JacobianInv.txt': self.jacobian_inv_matrix
        }
        
        for filename, expression in expressions.items():
            filepath = os.path.join(output_dir, filename)
            with open(filepath, 'w') as f:
                f.write(sp.pretty(expression, wrap_line=False))
    
    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Wrap angle to [-π, π] range."""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    @property
    def configuration_info(self) -> dict:
        """Get information about the current cluster configuration."""
        return {
            'num_robots': self.num_robots,
            'cluster_type': self.cluster_type.value,
            'control_mode': self.control_mode.value,
            'dof_per_robot': ROVER_DOF,
            'total_dof': self.num_robots * ROVER_DOF,
            'cluster_variables': len(self.cluster_angle_indices) if hasattr(self, 'cluster_angle_indices') else 0
        }


def main():
    """Example usage and testing."""
    # Create cluster with triangle leader configuration
    cluster = Cluster(
        num_robots=5, 
        cluster_type=ClusterConfig.TRILEAD,
        control_mode=ControlMode.POSITION
    )
    
    print("Cluster Configuration:", cluster.configuration_info)
    
    # Test desired position calculation
    desired_cluster_state = np.array([0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 4, 5, 0, 0.5, 0]).reshape(-1, 1)
    desired_positions = cluster.get_desired_robot_positions(desired_cluster_state)
    print("Desired robot positions:", desired_positions.flatten() if desired_positions is not None else None)
    
    # Export symbolic expressions
    cluster.export_symbolic_expressions()
    print("Symbolic expressions exported to 'cluster_expressions/' directory")


if __name__ == "__main__":
    main()