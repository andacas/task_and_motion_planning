(define (domain robot_bartender)
  (:requirements :strips :typing :equality :adl)

  (:types
    robot location section object - object
    hand - object
  )

  (:predicates
    ; Robot location predicates
    (at ?r - robot ?l - location)
    (hand-at ?h - hand ?s - section)
    (section-of ?s - section ?l - location)
    (free-hand ?h - hand)
    (connected ?l1 - location ?l2 - location)
  )

  ; Action: Move the robot from one location to another
  (:action move
    :parameters (?rob - robot ?from - location ?to - location)
    :precondition (and
                    (at ?rob ?from)
                    (connected ?from ?to))
    :effect (and
              (not (at ?rob ?from))
              (at ?rob ?to))
  )

  ; Action: Move a hand to a specific section
  (:action approach-section
    :parameters (?r - robot ?h - hand ?l - location ?s - section)
    :precondition (and
                    (at ?r ?l)
                    (free-hand ?h)
                    (section-of ?s ?l))
    :effect (hand-at ?h ?s)
  )

  ; Action: Retract a hand from a section
  (:action retract-hand
    :parameters (?h - hand ?s - section)
    :precondition (hand-at ?h ?s)
    :effect (not (hand-at ?h ?s))
  )
)
