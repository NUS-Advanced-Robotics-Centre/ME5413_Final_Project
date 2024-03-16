#import "template.typ": project

#show: project.with(
  title: "ME5413 Final Project Report",
  authors: (
    (name: "Cao Chenyu", 
    email: "E1192847@u.nus.edu", 
    ID: "A0285295N"),
    (name: "Li Zhangjin",
    email: "E1192649@u.nus.edu",
    ID: "A0285091B"),
    (name: "Zhao Xu",
    email: "E1192836@u.nus.edu",
    ID: "A0285284U"),
  ),
  logo: "./assets/logo-nus.png",
  abstract: lorem(59)
)

= Introduction
#lorem(60)

== In this paper
#lorem(20)

=== Contributions
#lorem(40)
#parbreak()
#lorem(30)

== Related Work
#lorem(500)
#lorem(500)

== More related work
#lorem(100)

= A New Life
#lorem(700)

= A New World
#lorem(20)
#parbreak()
#lorem(500)

== Check out the Headers
#lorem(150)
#lorem(150)

=== Do you like them?
#lorem(300)

// Bibliography section
#pagebreak(weak: true)
#set page(header: [])
= Bibliography
#lorem(30)