// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#ifndef QUICKBSP_VERTEX_LAYER_INL_H_
#define QUICKBSP_VERTEX_LAYER_INL_H_

#include <algorithm>
#include <cassert>
#include <fstream>
#include <vector>

#include "quickbsp/bsp.h"

#define BUF_LEN 1024

namespace graph {

/**
* @brief basic vertex type, edge type, structure type
*/

/**
* @brief which neighbours should receive message
*/
enum class GatherEdgeType{
  // don't generate message
  kNoEdges,
  // all in and out neighbours should receive message
  kAllEdges,
  // all in neighbours should receive message
  kAllInEdges,
  // all out neighbours should receive message
  kAllOutEdges
};

/**
 * @brief define node_id type
 */
typedef int node_id;

/**
 * @brief edge type
 * @type EdgeValue the type of value this edge carry, defined by users
 */
template <typename EdgeValue>
struct Edge {
  Edge(const node_id _source, const node_id _target, EdgeValue &&_edge_value)
      : source(_source),
        target(_target),
        edge_value(_edge_value) {}
  
  node_id source;
  node_id target;
  EdgeValue edge_value;
};

/**
 * @brief edge type with no edge value
 */
template<>
struct Edge<void> {
  Edge(const node_id _source, const node_id _target) 
      : source(_source),
        target(_target) {}
  
  node_id source;
  node_id target;
};

/**
 * @brief define EmptyEdge type
 */
typedef Edge<void> EmptyEdge;


/**
* @brief message type
* @type MessageValue the type of value this message carry, defined by users
*/
template<typename MessageValue>
struct VertexMessage {
  VertexMessage(const node_id target_, MessageValue &&message_value_) 
      : target(target_),
        message_value(message_value_) {}
  
  node_id target;
  MessageValue message_value;
};

/**
* @brief vertex type
* @type VertexValue the type of vertex data
* @type EdgeValue the type of edge data
* @type MessageValue the type of message data
*/
template<typename VertexValue, typename EdgeValue, typename MessageValue>
class BaseVertex {
 public:
  /**
   * @brief constructor
   */
  explicit BaseVertex(const node_id vertex_id) 
      : vertex_id_(vertex_id) {}

  /**
   * @brief get the real vertex id of this node
   */
  inline node_id vertex_id() const { return vertex_id_; }

  /**
   * @brief get the value of this node
   */
  inline const VertexValue& vertex_value() const { return vertex_value_; }
  
  /**
   * @brief set the value of this node
   * @param v the value to set
   */
  inline void set_vertex_value(VertexValue&& v) {
    vertex_value_ = std::move(v);
  }

  /**
   * @brief set the value of this node
   * @param v the value to set
   */
  inline void set_vertex_value(const VertexValue &v) {
    vertex_value_ = v;  
  }

  /**
   * @brief get accumulative message value
   * @return acc_message_value_
   */
  inline const MessageValue& acc_message_value() const { return acc_message_value_; }

  /**
   * @break set accumu message value
   * @param msg_value the value to set
   */
  inline void set_acc_message_value(MessageValue &&msg_value) {
    acc_message_value_ = std::move(msg_value);
  }

  inline void set_acc_message_value(const MessageValue &msg_value) {
    acc_message_value_ = msg_value;
  }

  /**
   * @brief get the start postition of the incoming edges of this node in the in edges array
   * @return the start position
   */
  inline int in_spos() const { return in_spos_; }
  
  /**
   * @brief set the start position of the incoming edges in the in edges array
   */
  inline void set_in_spos(const int spos) {
    in_spos_ = spos;
  }

  /**
   * @brief get the start position of the outgoing edges of this node in the out edges array
   * @return the start position
   */
  inline int out_spos() const { return out_spos_; }

  /**
   * @brief set the start position of the outgoing edges in the out edges array
   */
  inline void set_out_spos(const int spos) {
    out_spos_ = spos;
  }

  /**
   * @brief get the number of incoming edges of this node
   * @return the number of incoming edges
   */
  inline int in_len() const { return in_len_; }

  /**
   * @brief set the number of incoming edges
   * @param len number of incoming edges
   */
  inline void set_in_len(const int len) {
    in_len_ = len;
  }

  /**
   * @brief get the number of outgoing edges
   * @return number of outgoing edges
   */
  inline int out_len() const { return out_len_; }

  /**
   * @brief set the number of outgoing edges
   * @return number of outgoing edges
   */
  inline void set_out_len(const int len) {
    out_len_ = len;
  }
 
 private:
  // a unique real vertex id
  node_id vertex_id_;
  // vertex value
  VertexValue vertex_value_;
  // accumulative message value, used in message computation
  MessageValue acc_message_value_;
  // store the edges of all vertices in an array and use pos+len to get the edge positions in the array
  // start postition of incoming edges in the in edges array
  int in_spos_;
  // start position of outgoing edges in the out edges array
  int out_spos_;
  // number of incoming edges
  int in_len_;
  // number of outgoing edges
  int out_len_;
};

/**
 * @brief define graph type 
 * @type Vertex subclass of BaseVertex
 * @type EdgeValue type of edge value
 */
template<class Vertex, typename EdgeValue>
class Graph {
 public:
  /**
   * @brief constructor
   */
  Graph() : num_vertices_(0), num_edges_(0) {}

  /**
   * @brief clear the graph
   */
  void Clear() {
    num_vertices_ = 0;
    num_edges_ = 0;
    in_edges_.clear();
    out_edges_.clear();
    in_list_.clear();
    out_list_.clear();
    vertexid2index_map_.clear();
  }

  /**
   * @brief add new edge into the graph
   * @param source the source node id of this edge
   * @param target the target node id of this edge
   * @return 0 is edge is added, -1 if there is a duplicate edge in the graph
   */
  int AddNewEdge(const node_id source, const node_id target) {
    node_id source_index = AddNewNode(source);
    node_id target_index = AddNewNode(target);
    // only store the index in graph and the index will be used to send message
    // between vertices
    for (auto t : out_list_[source_index]) {
      if (t == target_index) {
        // there is duplicate edge, just return
        return -1;
      }
    }
    ++num_edges_;
    out_list_[source_index].push_back(target_index);
    in_list_[target_index].push_back(source_index);
    return 0;
  }

  /**
   * @brief finalize graph, the function will regenerate vertex and edge structure that is
   *    suitable for computation and release in_list_ and out_list_
   */
  void Finalize() {
    num_vertices_ = vertices_.size();
    in_edges_.reserve(num_edges_);
    out_edges_.reserve(num_edges_);

    // convert all the incoming edges of all vertex into a flat array(in_edges_)
    // and record position and length in every vertex
    int pos = 0;
    for (int i =  0; i < num_vertices_; ++i) {
      vertices_[i].set_in_spos(pos);
      vertices_[i].set_in_len(in_list_[i].size());
      for (auto edge_iter = in_list_[i].begin(); edge_iter != in_list_[i].end(); ++edge_iter) {
        in_edges_.push_back(*edge_iter);
        ++pos;
      }
    }
   
    // repeat for the outgoing edges
    pos = 0;
    for (int i = 0; i < num_vertices_; ++i) {
      vertices_[i].set_out_spos(pos);
      vertices_[i].set_out_len(out_list_[i].size());
      for (auto edge_iter = out_list_[i].begin(); edge_iter != out_list_[i].end(); ++edge_iter) {
        out_edges_.push_back(*edge_iter);
        ++pos;
       }
    } 

    // clear the in_list_ and out_list_
    for (auto iter = in_list_.begin(); iter != in_list_.end(); ++iter) {
      iter->clear();
      iter->shrink_to_fit();
    }
    in_list_.clear();
    in_list_.shrink_to_fit();
    for (auto iter = out_list_.begin(); iter != out_list_.end(); ++iter) {
      iter->clear();
      iter->shrink_to_fit();
    }
    out_list_.clear();
    out_list_.shrink_to_fit();
  }

  /**
   * @brief return the num of vertices in the graph
   * @return the num of vertices in the graph
   */
  inline int num_vertices() const { return num_vertices_; }

  /**
   * @brief get the vertex by vertex index
   * @param index the index of the vertex, the index should be >=0
   * @return the vertex
   */
  inline Vertex& GetVertexByIndex(const node_id index) {
    assert(index < num_vertices_);
    assert(index >= 0);
    return vertices_[index];
  }

  /**
   * @brief get the vertex index by the real node id
   * @param id the node id of the vertex
   * @return the index in the vertices, -1 if there is no such node id
   */
  inline node_id GetIndexByNodeId(const node_id id) {
    if (vertexid2index_map_.find(id) != vertexid2index_map_.end()) {
      return vertexid2index_map_[id];
    }
    return -1;
  }

  /**
   * @brief get the source node id of an edge by the index in the in_edges_
   * @return the source node id
   */
  inline node_id GetInEdgeByIndex(const node_id index) {
    assert(index < num_edges_);
    assert(index >= 0);
    return in_edges_[index];
  }

  /**
   * @brief get the target node id of an edge by the index in the out_edges_
   * @return the target node id
   */
  inline node_id GetOutEdgeByIndex(const node_id index) {
    assert(index < num_edges_);
    assert(index >= 0);
    return out_edges_[index];
  }

 private:
  /**
   * @brief add a new node to the graph
   * @param vertex_id the id of the vertex to be added, every node should have a 
   *       unique vertex_id. If there is already node with this vertex_id, just 
   *       return the index. otherwise add the node to the graph and return the 
   *       index.
   * @return the index in the vertices_
   */
  node_id AddNewNode(const node_id vertex_id) {
    if (vertexid2index_map_.find(vertex_id) == vertexid2index_map_.end()) {
      vertices_.push_back(std::move(Vertex(vertex_id)));
      in_list_.push_back(std::move(std::vector<node_id>()));
      out_list_.push_back(std::move(std::vector<node_id>()));
      vertexid2index_map_[vertex_id] = vertices_.size() - 1;
      return vertices_.size() - 1;
    } else {
      return vertexid2index_map_[vertex_id];
    }
  }

  // total vertices num
  int num_vertices_;
  // total edges num
  int num_edges_;
  // all the vertices
  std::vector<Vertex> vertices_;
  // in edges array
  std::vector<node_id> in_edges_;
  // out edges array
  std::vector<node_id> out_edges_;
  // tempory use, store the in_edges for every vertex, should not be used after finalized
  std::vector<std::vector<node_id> > in_list_;
  // tempory use, store the out_edges for every vertex, should not be used after finalized
  std::vector<std::vector<node_id> > out_list_;
  // map from real vertex id to index in the vertices_
  std::unordered_map<node_id, node_id> vertexid2index_map_;
};

/**
* @brief graph loader
*/
template<class Graph>
class GraphLoader {
 public:
  /**
   * @brief load graph with filename
   */
  Graph* Load(const char* filename) {
    std::cout << "open data file: " << filename << std::endl;
    std::ifstream fin(filename);
    char buf[BUF_LEN];
    fin.getline(buf, BUF_LEN);
    int v_num = atoi(buf);
    std::cout << "vertex num is: " << v_num << std::endl;

    Graph* graph_ptr = new Graph();
    // read line by line, every line should be : src_node_id dest_node_id
    while (!fin.eof()) {
      fin.getline(buf, BUF_LEN);
      if ('\0' == buf[0]) continue;
      const char* split = " ";
      char *p = nullptr;
      p = strtok(buf, split);
      int src_id = atoi(p);
      p = strtok(NULL, split);
      int dest_id = atoi(p);
      graph_ptr->AddNewEdge(src_id, dest_id);
    }
    return graph_ptr;
  }
};

}  // namespace graph

#endif  // QUICKBSP_VERTEX_LAYER_INL_H_
