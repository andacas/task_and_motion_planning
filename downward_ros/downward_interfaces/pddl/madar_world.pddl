(define (domain robot_bartender)
  (:requirements :strips :typing :equality :adl)

  (:types
    robot location section object - object
    hand - object
  )

  (:predicates
    (at ?r - robot ?l - location)
    (holding ?h - hand ?o - object)
    (hand-at ?h - hand ?s - section)
    (section-of ?s - section ?l - location)
    (free-hand ?h - hand)
    (connected ?l1 - location ?l2 - location)

    (left-accessible ?s - section)
    (right-accessible ?s - section)

    (arm-retracted ?h - hand)
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
                    (free-hand ?h)
                    (arm-retracted ?h)
                    (section-of ?s ?l))
    :effect (and
              (hand-at ?h ?s)
              (not (arm-retracted ?h)))
  )

  ; Retract a single arm from a section to retracted position
  ; No free-hand condition, can retract while holding an object
  (:action retractarm
    :parameters (?r - robot ?h - hand ?l - location ?s - section)
    :precondition (and
                    (at ?r ?l)
                    (hand-at ?h ?s))
    :effect (and
              (not (hand-at ?h ?s))
              (arm-retracted ?h))
  )

  ; Approach both arms to different sections simultaneously
  (:action approachsections2arms
    :parameters (?r - robot ?h1 - hand ?h2 - hand ?l - location ?s1 - section ?s2 - section)
    :precondition (and
                    (at ?r ?l)
                    (free-hand ?h1)
                    (free-hand ?h2)
                    (arm-retracted ?h1)
                    (arm-retracted ?h2)
                    (section-of ?s1 ?l)
                    (section-of ?s2 ?l)
                    (not (= ?h1 ?h2))
                    (not (= ?s1 ?s2))
                    (or (and (= ?h1 LEFT-HAND) (left-accessible ?s1))
                        (and (= ?h1 RIGHT-HAND) (right-accessible ?s1)))
                    (or (and (= ?h2 LEFT-HAND) (left-accessible ?s2))
                        (and (= ?h2 RIGHT-HAND) (right-accessible ?s2))))
    :effect (and
              (hand-at ?h1 ?s1)
              (hand-at ?h2 ?s2)
              (not (arm-retracted ?h1))
              (not (arm-retracted ?h2)))
  )

  ; Pick an object with a hand
  (:action pick
    :parameters (?r - robot ?h - hand ?o - object ?s - section)
    :precondition (and
                    (free-hand ?h)
                    (hand-at ?h ?s)
                    (at ?o ?s)
                    (or (and (= ?h LEFT-HAND) (left-accessible ?s))
                        (and (= ?h RIGHT-HAND) (right-accessible ?s))))
    :effect (and
              (not (free-hand ?h))
              (holding ?h ?o)
              (not (at ?o ?s)))
  )
)
