# NASA Python Naming Conventions

NASA's F´ (F Prime) flight software framework follows standard Python conventions (PEP 8) with additional emphasis on safety-critical code readability and maintainability.

## Core Principles

- **Readability first**: Code is read more often than written
- **Consistency**: Use uniform naming throughout the codebase  
- **Descriptive names**: Names should clearly indicate purpose
- **Safety-critical mindset**: Prioritize clarity over brevity

## Variables

### Standard Variables
- Use `snake_case` (lowercase with underscores)
- Be descriptive and unambiguous

```python
# Good
user_count = 0
telemetry_data = []
spacecraft_altitude = 1000.5

# Bad  
n = 0
data = []
alt = 1000.5
```

### Constants
- Use `SCREAMING_SNAKE_CASE` (uppercase with underscores)

```python
# Good
MAX_RETRY_ATTEMPTS = 3
EARTH_RADIUS_KM = 6371.0
DEFAULT_TIMEOUT_SECONDS = 30

# Bad
max_retry = 3
earthRadius = 6371.0
```

### Private Variables
- Prefix with single underscore `_`
- Use for internal class/module variables

```python
class SpacecraftController:
    def __init__(self):
        self._internal_state = "idle"
        self._error_count = 0
```

### Protected Variables (Name Mangling)
- Prefix with double underscore `__` when name mangling needed

```python
class FlightComputer:
    def __init__(self):
        self.__critical_data = {}  # Name mangled to _FlightComputer__critical_data
```

## Functions

### Function Names
- Use `snake_case`
- Start with verb when function performs action
- Be descriptive about what the function does

```python
# Good
def calculate_orbital_velocity(altitude, planet_mass):
    pass

def validate_telemetry_packet(packet):
    pass

def initialize_flight_systems():
    pass

# Bad
def calc(a, m):
    pass

def check(p):
    pass

def init():
    pass
```

### Function Parameters
- Use `snake_case`
- Avoid single letters except for mathematical contexts
- Use descriptive names

```python
# Good
def compute_trajectory(initial_position, target_coordinates, fuel_mass):
    pass

# Acceptable for mathematical functions
def distance_formula(x1, y1, x2, y2):
    pass

# Bad
def compute_trajectory(pos, tgt, f):
    pass
```

## Classes

### Class Names
- Use `PascalCase` (CapWords)
- Use noun phrases
- Be specific about the class purpose

```python
# Good
class TelemetryProcessor:
    pass

class SpacecraftAttitudeController:
    pass

class FlightDataRecorder:
    pass

# Bad
class processor:
    pass

class Controller:  # Too generic
    pass

class data:
    pass
```

### Method Names
- Use `snake_case`
- Follow same rules as functions

```python
class MissionPlanner:
    def calculate_burn_sequence(self):
        pass
    
    def validate_mission_parameters(self):
        pass
    
    def _internal_helper_method(self):  # Private method
        pass
```

## Modules and Packages

### Module Names
- Use `snake_case`
- Short, descriptive names
- Avoid underscores if possible

```python
# Good
telemetry.py
flight_control.py
navigation.py

# Acceptable
attitude_control.py

# Bad
TelemetryModule.py
flight-control.py
```

### Package Names
- Use `lowercase`
- Avoid underscores
- Short names

```python
# Good
spacecraft/
mission/
utils/

# Bad
SpaceCraft/
mission_planning/
```

## Special Naming Rules

### Boolean Variables
- Use descriptive names that clearly indicate true/false meaning
- Prefix with `is_`, `has_`, `can_`, `should_` when appropriate

```python
# Good
is_system_initialized = True
has_valid_telemetry = False
can_execute_maneuver = True
system_ready = True

# Bad
flag = True
check = False
```

### Collections
- Use plural nouns for collections
- Be specific about contents

```python
# Good
sensor_readings = []
active_components = {}
error_messages = set()

# Bad
data = []
items = {}
stuff = set()
```

## Avoided Characters

Never use these as single character variable names:
- `l` (lowercase L) - looks like 1
- `O` (uppercase O) - looks like 0  
- `I` (uppercase i) - looks like 1

```python
# Bad
l = [1, 2, 3]
O = {"key": "value"}
I = 42

# Good
readings = [1, 2, 3]
config = {"key": "value"}
index = 42
```

## NASA-Specific Considerations

### Flight Software Principles
- Prioritize readability over cleverness
- Use full words instead of abbreviations when unclear
- Consider that code may be reviewed by mission-critical safety teams

```python
# Good - Clear for safety review
def validate_spacecraft_orientation():
    current_attitude = get_attitude_data()
    if not is_attitude_within_limits(current_attitude):
        trigger_attitude_correction()

# Bad - Abbreviations unclear in safety context  
def val_sc_orient():
    att = get_att()
    if not chk_lim(att):
        corr()
```

### Documentation Integration
- Use names that work well with docstrings
- Consider that names will appear in generated documentation

```python
def calculate_delta_v_for_maneuver(
    current_velocity: float,
    target_velocity: float,
    spacecraft_mass: float
) -> float:
    """
    Calculate delta-v required for spacecraft maneuver.
    
    Args:
        current_velocity: Current spacecraft velocity in m/s
        target_velocity: Desired velocity after maneuver in m/s  
        spacecraft_mass: Total spacecraft mass in kg
        
    Returns:
        Required delta-v in m/s
    """
    pass
```

## Tools and Enforcement

### Required Tools
- **Black**: Automatic code formatting
- **Pylama**: Static analysis (includes pylint, pyflakes, pycodestyle)
- **Pre-commit hooks**: Automatic formatting on commit

### Configuration
Use NASA F´'s pylama configuration for consistency with flight software standards.

---

*This guide follows PEP 8 standards with NASA F´ framework practices for safety-critical flight software development.*
