# Cloud Service Thread Safety Design

Date: 2026-04-26

## Problem

`CloudService` currently has a confirmed data race on `attr_cb_`.

- `subscribe_shared_attributes()` writes `attr_cb_` while holding `rpc_mtx_`
- the network-delivery lambda reads and invokes `attr_cb_` without taking that mutex
- `main.cc` connects the network before registering the callback, so the read/write overlap window is real

This is live undefined behavior in production code, not a theoretical cleanliness issue.

## Scope

In scope:

- `CloudService` callback publication and invocation
- callback registration order assumptions
- any minimal API or implementation changes needed to make callback access structurally safe

Out of scope:

- RPC handler redesign
- telemetry publishing changes
- unrelated `NetworkManager` or transport refactors

## Proposed Changes

Use a single ownership and synchronization model for shared-attribute callbacks.

### 1. Make callback publication and use follow one rule

Recommended approach:

- guard both callback registration and callback snapshotting with the same mutex
- in the network callback, take the mutex briefly, copy or move a local callable snapshot, release the lock, then invoke the snapshot outside the lock

This avoids:

- data races on `std::function`
- invoking user callback while holding `rpc_mtx_`
- lock inversion with callback code that may call back into config or cloud-related paths

### 2. Separate RPC handler locking from attribute callback locking if needed

If `rpc_mtx_` becomes conceptually overloaded, split it into:

- `rpc_mtx_` for RPC handler map
- `attr_cb_mtx_` for shared-attribute callback publication

This is preferable if keeping one mutex would make the class harder to reason about.

### 3. Remove reliance on registration timing

The implementation should be correct regardless of whether:

- network messages arrive before callback registration
- callback registration happens once at startup
- callback is replaced later at runtime

If no callback is registered, the code should safely drop the message with no race and no side effects.

## Risks and Tradeoffs

- Keeping one mutex is a smaller patch, but it couples two unrelated synchronization domains.
- Splitting the mutexes is clearer, but slightly larger in scope.
- Copying a local callback snapshot adds one small callable copy per attribute message, but that is an acceptable tradeoff for correctness.

## Testing and Verification

Required verification should cover:

- no functional regression in existing RPC behavior
- shared-attribute callback still fires after registration
- safe behavior when attribute messages arrive before registration
- safe behavior when callback is absent

If unit tests cannot deterministically prove race freedom, they should still validate the structural change:

- registration path stores callback
- delivery path snapshots callback before invocation
- callback invocation does not require holding the same mutex used for registration

## Non-Goals

- redesigning `CloudService` into a general event dispatcher
- changing transport threading models
- adding attribute-message buffering or replay
