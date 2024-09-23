# This PackML status message captures the PackML state machine data.  It's
# meant to be published continuously or latched/on changes (depending on
# implementation)
# Based on PackML summarized here:
# http://www.plcopen.org/pages/promotion/publications/downloads/mapping_omac_statediagram.pdf


State state         # standard PackML State

Mode mode           # standard PackML Mode

# TODO: Do we need this?
int16 error         # TODO: define standard errors (probably in a separate msg)
int16 sub_error     # non-standard/user defined error code