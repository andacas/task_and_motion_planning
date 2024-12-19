(define (domain robot_bartender)
  (:requirements :strips :typing :equality :adl)

  (:types
    robot location section object - object
    hand - object
  )

  (:predicates
    (at ?r - robot ?l - location)              ; Robot at location
    (holding ?h - hand ?o - object)           ; Hand holding an object
    (obj-at ?o - object ?s - section)         ; Object at section
    (hand-at ?h - hand ?s - section)          ; Hand at section
    (section-of ?s - section ?l - location)   ; Section belongs to location
    (free-hand ?h - hand)                     ; Hand is free
    (connected ?l1 - location ?l2 - location) ; Locations are connected
    (left-accessible ?s - section)            ; Section accessible by left hand
    (right-accessible ?s - section)           ; Section accessible by right hand
    (arm-retracted ?h - hand)                 ; Arm is retracted
    (glass-empty ?o - object)                 ; Glass is empty
    (glass-filled ?o - object)                ; Glass is filled (drink ready)
  )

  ; Move action
  (:action move
    :parameters (?rob - robot ?from - location ?to - location)
    :precondition (and
                    (at ?rob ?from)
                    (connected ?from ?to)
                    (arm-retracted LEFT-HAND)
                    (arm-retracted RIGHT-HAND))
    :effect (and
              (not (at ?rob ?from))
              (at ?rob ?to))
  )

  ; Approach section action
  (:action approach_section
    :parameters (?r - robot ?h - hand ?l - location ?s - section)
    :precondition (and
                    (at ?r ?l)
                    (section-of ?s ?l)
                    (arm-retracted LEFT-HAND)
                    (arm-retracted RIGHT-HAND))
    :effect (and
              (hand-at ?h ?s)
              (not (arm-retracted ?h)))
  )

  ; Retract arm action
  (:action retractarm
    :parameters (?r - robot ?h - hand ?l - location ?s - section)
    :precondition (and
                    (at ?r ?l)
                    (hand-at ?h ?s))
    :effect (and
              (not (hand-at ?h ?s))
              (arm-retracted ?h))
  )

  ; Pick action
  (:action pick
    :parameters (?r - robot ?h - hand ?o - object ?s - section)
    :precondition (and
                    (free-hand ?h)
                    (hand-at ?h ?s)
                    (obj-at ?o ?s))
    :effect (and
              (not (free-hand ?h))
              (holding ?h ?o)
              (not (obj-at ?o ?s)))
  )

  ; Place action
  (:action place
    :parameters (?r - robot ?h - hand ?o - object ?s - section)
    :precondition (and
                    (holding ?h ?o)
                    (hand-at ?h ?s))
    :effect (and
              (free-hand ?h)
              (not (holding ?h ?o))
              (obj-at ?o ?s))
  )

  ; Prepare drink action
  (:action prepare_drink
    :parameters (?r - robot ?o - object  ?s - section)
    :precondition (and
                    (at ?r PREPARATION-TABLE) 
                    (obj-at ?o ?s)
                    (glass-empty ?o)
                    (free-hand LEFT-HAND)
                    (free-hand RIGHT-HAND)
                    (arm-retracted LEFT-HAND)
                    (arm-retracted RIGHT-HAND)
                    (section-of ?s PREPARATION-TABLE))
    :effect (and
              (not (glass-empty ?o))
              (glass-filled ?o))
  )
)
