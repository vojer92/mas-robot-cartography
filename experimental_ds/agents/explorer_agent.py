from agents.base_agent import BaseAgent

from environment.multi_grid_with_properties import MultiGridWithProperties

# single activities
from algorithms.environment_perception.environment_perception_method import EnvironmentPerceptionMethod
from algorithms.movement_goal_finding.movement_goal_finding_method import MovementGoalFindingMethod
from algorithms.communication.communication_method import CommunicationMethod
from algorithms.movement_goal_selection.movement_goal_selection_method import MovementGoalSelectionMethod
from algorithms.cluster_processing.cluster_processing_method import ClusterProcessingMethod
from algorithms.movement_route_finding.movement_route_finding_method import MovementRouteFindingMethod
# ...

class ExplorerAgent(BaseAgent):
    """
    Base class for all agents.
    Expects initialized activities in constructor (from factory)
    """
    def __init__(self,
        local_grid_width: int, # Width of the environment to create a fitting local memory
        local_grid_height: int, # Height of the environment to create a fitting local memory
        torus: bool,
        environment_perception_method: EnvironmentPerceptionMethod = None,
        movement_goal_finding_method: MovementGoalFindingMethod = None,
        local_grid_communication_method: CommunicationMethod = None,
        cluster_processing_method: ClusterProcessingMethod = None,
        movement_goal_selection_method: MovementGoalSelectionMethod = None,
        movement_route_finding_method: MovementRouteFindingMethod = None,
        movement_collision_avoidance_method: MovementRouteFindingMethod = None,
        *args,
        **kwargs
    ) -> None:
        super().__init__(*args, **kwargs)
        # Initialize a local memory for the environment
        self.local_grid = MultiGridWithProperties(
            local_grid_width,
            local_grid_height,
            torus
        )
        self.local_grid.add_property("perception_time")
        # Initialize additional parameters
        self.orientation = 270
        self.step_counter = 0
        # Initialize the single activities
        self.environment_perception_method = environment_perception_method
        self.movement_goal_finding_method = movement_goal_finding_method
        self.local_grid_communication_method = local_grid_communication_method
        self.cluster_processing_method = cluster_processing_method
        self.movement_goal_selection_method = movement_goal_selection_method
        self.movement_route_finding_method = movement_route_finding_method
        self.movement_collision_avoidance_method = movement_collision_avoidance_method

    def step(self) -> None:
        # TODO:
        #  Muss ausgearbeitet werden (spezifisch in den Agenten-Typen)!
        #  Aktuell quasi Pseudocode der bereits implementierten Methoden fuer die Uebersicht.
        #  Wichtig! Lokale Sicht und Autonomie muss erhalten bleiben (Skalierbarkeit, Effizient, Emergenz),
        #  also Kommunikation und Koordination nur alle n Schritte oder bei wichtigen Ereignissen!

        # Perception of the environment; In every step for collision avoidance
        self.environment_perception_method.scan_environment(
            world = self.model.grid,
            current_time=self.model.steps,
            agent_pos=self.pos,
            agent_orientation=self.orientation,
            agent_local_memory = self.local_grid,
            blocked_by_objects = True
        )

        # Communication; When e.g. goal is reached or every n steps, but not in every step to keep efficiency and autonomy
        self.local_grid_communication_method.send(
            key="global_map",
            value=self.local_grid,
        )


        # Coordination
        #  Check for new coordination instructions in every step!  Maybe via a coordination_necessary-flag

        # Find potential movement goals; During coordination or as fallback after failed coordination attempt and without current goal
        movement_goals = self.movement_goal_finding_method.find_goals(
            self.local_grid,
            moore
        )

        # Select movement goal; After coordination (get coordinated goal from assignment) or from fallback
        selected_goal_pos = self.movement_goal_selection_method.select_movement_goal(
            self.local_grid,
            self.pos,
            movement_goals,
            agent = self,
            model = self.model,
        )

        # Calculate route to movement goal; After selection of new movement goal
        if selected_goal_pos is not None:
            path = self.movement_route_finding_method.find_path(
                self.local_grid,
                self.pos,
                selected_goal_pos,
                moore,
                self.model.steps
            )

        else:
            # Without a goal no path is calculated
            path = None

        # Move agent; Every step
        if path:
            next_pos = path[0]

            # Collision avoidance; Check before execution of every movement
            self.movement_collision_avoidance_method.find_path(
                self.local_grid,
                self.pos,
                next_pos,
                moore,
                self.model.steps,
            )

            # TODO:
            #  Pruefung, ob next_pos innerhalb einer Schrittweite erreichbar ist, erforderlich!
            # TODO:
            #  Use move and turn functions from BaseAgent
            if not self.pos == next_pos:
                self.model.grid.move_agent(self, next_pos)
                self.step_counter += 1

        else:
            # Without a goal agents remain on position / maybe do a 360-degree turn once to check close surroundings if not already known
            pass # explicit not move (usable for logging)