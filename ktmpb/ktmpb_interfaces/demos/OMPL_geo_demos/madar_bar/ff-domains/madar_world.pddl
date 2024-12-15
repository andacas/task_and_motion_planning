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
  )

  ; Move requires both arms retracted
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

  ; Approach a single section with one arm
  (:action approach_section
    :parameters (?r - robot ?h - hand ?l - location ?s - section)
    :precondition (and
                    (at ?r ?l)
                    (arm-retracted ?h)
                    (section-of ?s ?l))
    :effect (and
              (hand-at ?h ?s)
              (not (arm-retracted ?h)))
  )

  ; Retract a single arm
  (:action retractarm
    :parameters (?r - robot ?h - hand ?l - location ?s - section)
    :precondition (and
                    (at ?r ?l)
                    (hand-at ?h ?s))
    :effect (and
              (not (hand-at ?h ?s))
              (arm-retracted ?h))
  )

  ; Pick an object with a hand
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

  ; Place an object with a hand
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
)
