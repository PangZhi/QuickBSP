// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include "quickbsp/vertex_layer-inl.h"
#include "quickbsp/engine-inl.h"

#define BUF_LEN 1024

struct VertexValue {
  explicit VertexValue(int d) : dist(d) {}
  VertexValue() : dist(10000) {};
  int dist;
};


class MVertex : public graph::BaseVertex<VertexValue, void, int> {
 public:
  MVertex(const graph::node_id vertex_id) : BaseVertex(vertex_id) {}
  
  void Compute(int message_value) {
    if (this->acc_message_value() > message_value + 1) {
      this->set_acc_message_value(message_value + 1);
    }
  }

  void Reset() {
    this->set_acc_message_value(10000);
  }

  double GetMessageValue() {
    return this->vertex_value().dist;
  }

  void UpdateValue() {
    if (this->vertex_value().dist > this->acc_message_value()) {
      this->set_vertex_value(VertexValue(this->acc_message_value()));
      is_update_ = true;
    } else {
      is_update_ = false;
    }
    this->set_acc_message_value(10000);
  }

  graph::GatherEdgeType GetSendEdgeType() const {
    if (is_update_) {
      return graph::GatherEdgeType::kAllOutEdges;
    } else {
      return graph::GatherEdgeType::kNoEdges;
    }
  }

 private:
  bool is_update_;
};

typedef graph::Edge<void> MEdgeType;

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "ERROR: argument length should be 3" << std::endl;
    exit(-1);
  }

  const char* filename = argv[1];
  int thread_num = atoi(argv[2]);

  typedef graph::Graph<MVertex, void> GraphType;
  GraphType* graph_ptr = (new graph::GraphLoader<GraphType>)->Load(filename);
  graph_ptr->Finalize();
  
  for (int i = 0 ; i < graph_ptr->num_vertices(); i ++) {
    graph_ptr->GetVertexByIndex(i).Reset();
  }
  graph_ptr->GetVertexByIndex(0).set_acc_message_value(0);
    
  typedef bsp::Engine<GraphType, MVertex, MEdgeType, double> Engine;
  std::unique_ptr<Engine> engine_ptr 
      = std::unique_ptr<Engine>(new Engine(thread_num,
                                           graph_ptr,
                                           bsp::PartitionType::kPartitionRoundRobin,
                                           20));
  engine_ptr->Run();
  for (int i = 0 ; i < 100 ; i ++) {
    std::cout << graph_ptr->GetVertexByIndex(graph_ptr->GetIndexByNodeId(i)).vertex_value().dist << " ";
  }
}
