# GUID of the tracked item
string guid

# ID or name of the entity holding onto this item
string holding_entity

# Types of holding entities
uint8 TRANSPORTER=0
uint8 WORKCELL = 1
uint8 UNKNOWN = 2

# Type of entity holding onto this item
uint8 holding_entity_type

# The stringified yaml of the item's model, including it's metadata
string item_model_yaml
