/*
 * Copyright (c) 2013, Peter Rutenbar <pruten@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "shoebill.h"
#include "redblack.h"

// Create a new red-black tree 
// (just return an empty black leaf pointer)
rb_tree* rb_new()
{
    return calloc(sizeof(rb_tree), 1);
}

// Insert a new key/value into the tree 
// (and return the old value if *old_value is non-null.)
// Returns true if the key already existed.
uint8_t rb_insert(rb_tree *root, rb_key_t key, rb_value_t value, rb_value_t *old_value)
{    
    // Special edge case: insert the root node if tree's empty
    if (*root == NULL) {
        *root = calloc(sizeof(rb_node), 1);
        (*root)->key = key;
        (*root)->value = value;
        return 0;
    }
    
    // traverse
    rb_node **cur = root, *parent = NULL;
    while (*cur) {
        parent = *cur;
        if (key < (*cur)->key) // left
            cur = &((*cur)->left);
        else if (key > (*cur)->key)  // right
            cur = &((*cur)->right);
        else { // the key already exists
            if (old_value) 
                *old_value = (*cur)->value;
            (*cur)->value = value;
            return 1; // 1 => the key already existed
        }
    }
    
    // insert
    *cur = calloc(sizeof(rb_node), 1);
    (*cur)->parent = parent;
    (*cur)->key = key;
    (*cur)->value = value;
    (*cur)->is_red = 1;
    
    // resolve
    
    rb_node *red = *cur;
    while (red) {
        rb_node *parent = red->parent;
        if (red->is_red == 0) // if this node isn't actually red, return
            break;
        else if (!parent)  // if this is the root node, return
            break;
        else if (!parent->is_red) // if the parent is black, then we're done.
            break;
        
        // otherwise, the parent is red - the grandparent must exist, and must be black
        assert((red->parent->parent) && (!red->parent->parent->is_red));
        
        rb_node *gparent = parent->parent;
        rb_node *uncle = (gparent->left==parent)?gparent->right:gparent->left;
        
        // 8 cases:
        // LLr LRr RLr RRr (uncle is red)
        if (uncle && (uncle->is_red)) {
            uncle->is_red = 0;
            parent->is_red = 0;
            gparent->is_red = 1;
            red = gparent;
            continue ;
        }
        
        // uncle is black
        
        rb_node **gparent_ptr;
        if (gparent->parent) { // great-grandparent exists
            rb_node *ggparent = gparent->parent;
            gparent_ptr = (ggparent->left==gparent)? (&ggparent->left) : (&ggparent->right);
        } else { // grandparent is root
            gparent_ptr = root;
        }
        
        uint8_t mycase = ((gparent->left==parent)?0:1);
        mycase = (mycase << 1) | ((parent->left==red)?0:1);
        switch (mycase) {
            
            case 0: {// LLb
                rb_node *Br = parent->right;
                *gparent_ptr = parent;
                parent->right = gparent;
                gparent->left = Br;
                parent->is_red = 0;
                gparent->is_red = 1;
                
                parent->parent = gparent->parent;
                gparent->parent = parent;
                if (Br) Br->parent = gparent;

                red = gparent->right; // gparent became red, gparent->left is black, check gparent->right
                break ;
            }
            case 1: {// LRb 
                rb_node *Cl = red->left;
                rb_node *Cr = red->right;
                *gparent_ptr = red;
                red->left = parent;
                red->right = gparent;
                parent->right = Cl;
                gparent->left = Cr;
                red->is_red = 0;
                gparent->is_red = 1;
                
                red->parent = gparent->parent;
                parent->parent = red;
                gparent->parent = red;
                if (Cl) Cl->parent = parent;
                if (Cr) Cr->parent = gparent;
                
                red = gparent->right;
                break ;
            }
            case 2: { // RLb
                rb_node *Cr = red->right, *Cl = red->left;
                *gparent_ptr = red;
                red->left = gparent;
                red->right = parent;
                gparent->right = Cl;
                parent->left = Cr;
                red->is_red = 0;
                gparent->is_red = 1;
                
                
                red->parent = gparent->parent;
                gparent->parent = red;
                parent->parent = red;
                if (Cr) Cr->parent = parent;
                if (Cl) Cl->parent = gparent;

                red = gparent->left;
                break;
            }
            case 3: { // RRb
                rb_node *Bl = parent->left;
                *gparent_ptr = parent;
                parent->left = gparent;
                gparent->right = Bl;
                parent->is_red = 0;
                gparent->is_red = 1;
                
                parent->parent = gparent->parent;
                gparent->parent = parent;
                if (Bl) Bl->parent = gparent;
                
                red = gparent->left;
                break;
            }
        }
    }
        
    (*root)->is_red = 0; // make double-sure root is red
    return 0;
}

// Find a value given a key
uint8_t rb_find (rb_tree *tree, rb_key_t key, rb_value_t *value)
{
    rb_node *cur = *tree;
    
    while (cur) {
        if (key < cur->key) 
            cur = cur->left;
        else if (key > cur->key)
            cur = cur->right;
        else {
            if (value)
                *value = cur->value;
            return 1;
        }
    }
    return 0;
}

uint8_t _rb_index (rb_node *cur, uint32_t *index, rb_node **result)
{
    if (!cur) 
        return 0;

    else if (_rb_index(cur->left, index, result) == 1) 
        return 1;

    else if (0 == *index) {
        *result = cur;
        return 1;
    }
    --*index;

    return _rb_index(cur->right, index, result);
}

// Do an in-order traversal, and retrieve the (index)th sorted key/value
uint8_t rb_index (rb_tree *tree, uint32_t index, rb_key_t *key, rb_value_t *value)
{
    rb_node *cur = *tree, *result;    
    if (_rb_index(cur, &index, &result)) {
        if (key) *key = result->key;
        if (value) *value = result->value;
        return 1;
    }
    return 0;
}

// Count the number of nodes in the tree
uint32_t rb_count (rb_tree *tree)
{
    rb_node *node = *tree;
    if (!node) 
        return 0;
    return 1 + rb_count(&node->left) + rb_count(&node->right);
}

void _rb_free (rb_node *node)
{
    if (!node) return ;
    _rb_free(node->right);
    if (node->right) free(node->right);
    _rb_free(node->left);
    if (node->left) free(node->left);
}
    
// Free all the nodes (and the rb_tree ptr itself)
void rb_free (rb_tree *tree)
{
    _rb_free(*tree);
    free(*tree);
    free(tree);
}

